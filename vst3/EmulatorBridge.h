#ifndef EPS16_VST3_EMULATOR_BRIDGE_H
#define EPS16_VST3_EMULATOR_BRIDGE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace eps16::vst3 {

constexpr std::uint64_t kCpuClockHz = 10000000;

struct MidiEvent {
    int sampleOffset{};
    std::uint8_t status{};
    std::uint8_t data1{};
    std::uint8_t data2{};
    const std::uint8_t *bytes{};
    std::size_t size{};
};

struct SysExOutputEvent {
    static constexpr std::size_t capacity = 512;
    int sampleOffset{};
    std::size_t size{};
    std::array<std::uint8_t, capacity> bytes{};
};

class EmulatorSink {
public:
    virtual ~EmulatorSink() = default;
    virtual bool beginBlock() { return true; }
    virtual void endBlock() {}
    virtual void prepare(double dawSampleRate) = 0;
    virtual void runUntil(std::uint64_t absoluteCpuCycle) = 0;
    virtual void midi(std::uint8_t status, std::uint8_t data1,
                      std::uint8_t data2, std::uint64_t cycle) = 0;
    virtual void keyboard(std::uint8_t note, std::uint8_t velocity,
                          bool pressed, std::uint64_t cycle) = 0;
    virtual void midiBytes(const std::uint8_t *bytes, std::size_t size,
                           std::uint64_t cycle) = 0;
    virtual std::size_t drainMidiOutput(std::uint8_t *bytes,
                                        std::size_t capacity) = 0;
    virtual void panelByte(std::uint8_t value, std::uint64_t cycle) = 0;
    virtual void analog(unsigned int channel, std::uint16_t value,
                        std::uint64_t cycle) = 0;
    virtual void samplingInput(float left, float right, std::uint64_t cycle) = 0;
    virtual void audioOutput(float *channels, std::size_t channelCount,
                             std::uint64_t cycle) = 0;
};

class SilentSink final : public EmulatorSink {
public:
    void prepare(double) override {}
    void runUntil(std::uint64_t) override {}
    void midi(std::uint8_t, std::uint8_t, std::uint8_t, std::uint64_t) override {}
    void keyboard(std::uint8_t, std::uint8_t, bool, std::uint64_t) override {}
    void midiBytes(const std::uint8_t *, std::size_t, std::uint64_t) override {}
    std::size_t drainMidiOutput(std::uint8_t *, std::size_t) override {
        return 0;
    }
    void panelByte(std::uint8_t, std::uint64_t) override {}
    void analog(unsigned int, std::uint16_t, std::uint64_t) override {}
    void samplingInput(float, float, std::uint64_t) override {}
    void audioOutput(float *channels, std::size_t channelCount,
                     std::uint64_t) override {
        std::fill_n(channels, channelCount, 0.0f);
    }
};

class DawClock {
public:
    bool prepare(double sampleRate);
    std::uint64_t advanceOneSample();
    [[nodiscard]] std::uint64_t cycles() const { return totalCycles; }
    [[nodiscard]] std::uint64_t rate() const { return sampleRateHz; }

private:
    std::uint64_t sampleRateHz{};
    std::uint64_t remainder{};
    std::uint64_t totalCycles{};
};

class HostMidiClock {
public:
    bool prepare(double sampleRate);
    void reset();
    std::size_t generate(bool positionValid, bool playing, double bpm,
                         double ppqPosition, int samples, MidiEvent *events,
                         std::size_t capacity);

private:
    double sampleRate{};
    double expectedPpq{};
    std::int64_t nextClockTick{};
    bool transportKnown{};
    bool wasPlaying{};
};

class EmulatorBridge {
public:
    static constexpr std::size_t controlQueueCapacity = 512;
    static constexpr std::size_t sysExQueueCapacity = 8;
    static constexpr std::size_t sysExMessageCapacity = 128;

    explicit EmulatorBridge(EmulatorSink &sinkToUse) : sink(sinkToUse) {}

    bool prepare(double sampleRate);
    bool resetTimeline();
    bool enqueuePanelTransition(std::uint8_t rawMatrixCode, bool pressed);
    bool enqueueKeyboardTransition(std::uint8_t note, std::uint8_t velocity,
                                   bool pressed);
    bool enqueueAnalog(unsigned int channel, std::uint16_t value);
    bool enqueueSysEx(const std::uint8_t *bytes, std::size_t size);
    void process(const float *inputLeft, const float *inputRight,
                 float *const *outputChannels, std::size_t outputChannelCount,
                 int samples,
                 const MidiEvent *midiEvents, std::size_t midiEventCount,
                 SysExOutputEvent *sysExOutput = nullptr,
                 std::size_t sysExOutputCapacity = 0,
                 std::size_t *sysExOutputCount = nullptr);
    void process(const float *inputLeft, const float *inputRight,
                 float *outputLeft, float *outputRight, int samples,
                 const MidiEvent *midiEvents, std::size_t midiEventCount,
                 SysExOutputEvent *sysExOutput = nullptr,
                 std::size_t sysExOutputCapacity = 0,
                 std::size_t *sysExOutputCount = nullptr) {
        float *channels[2] = {outputLeft, outputRight};
        process(inputLeft, inputRight, channels, 2, samples, midiEvents,
                midiEventCount, sysExOutput, sysExOutputCapacity,
                sysExOutputCount);
    }

    [[nodiscard]] std::uint64_t cpuCycles() const {
        return publishedCycles.load(std::memory_order_relaxed);
    }
    [[nodiscard]] std::uint64_t droppedControls() const {
        return controlDrops.load(std::memory_order_relaxed);
    }

private:
    static constexpr std::uint64_t panelTransitionSpacingCycles = 500000;
    enum class ControlType : std::uint8_t { panel, analog, keyboard };
    struct ControlEvent {
        ControlType type{};
        std::uint8_t first{};
        std::uint16_t second{};
    };
    struct QueuedSysEx {
        std::array<std::uint8_t, sysExMessageCapacity> bytes{};
        std::size_t size{};
    };
    bool enqueue(ControlEvent event);
    bool dequeue(ControlEvent &event);
    bool dequeueSysEx(QueuedSysEx &message);
    void dispatchControls(std::uint64_t cycle);
    void dispatchMidi(const MidiEvent &event, std::uint64_t cycle);
    void collectMidiOutput(int sampleOffset, SysExOutputEvent *output,
                           std::size_t capacity, std::size_t &count);

    EmulatorSink &sink;
    DawClock clock;
    std::array<ControlEvent, controlQueueCapacity> controls{};
    std::atomic<std::size_t> controlRead{};
    std::atomic<std::size_t> controlWrite{};
    std::atomic<std::uint64_t> controlDrops{};
    std::atomic<std::uint64_t> publishedCycles{};
    std::array<QueuedSysEx, sysExQueueCapacity> queuedSysEx{};
    std::atomic<std::size_t> sysExRead{};
    std::atomic<std::size_t> sysExWrite{};
    ControlEvent pendingControl{};
    bool hasPendingControl{};
    std::uint64_t nextPanelTransitionCycle{};
    std::array<std::uint8_t, SysExOutputEvent::capacity> outputSysEx{};
    std::size_t outputSysExSize{};
    bool outputInSysEx{};
};

} // namespace eps16::vst3

#endif
