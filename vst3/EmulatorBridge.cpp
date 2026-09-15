#include "EmulatorBridge.h"

#include <algorithm>
#include <cmath>

namespace eps16::vst3 {

bool DawClock::prepare(double sampleRate) {
    if (!std::isfinite(sampleRate) || sampleRate < 8000.0 || sampleRate > 768000.0)
        return false;
    sampleRateHz = static_cast<std::uint64_t>(std::llround(sampleRate));
    remainder = 0;
    totalCycles = 0;
    return sampleRateHz != 0;
}

std::uint64_t DawClock::advanceOneSample() {
    remainder += kCpuClockHz;
    totalCycles += remainder / sampleRateHz;
    remainder %= sampleRateHz;
    return totalCycles;
}

bool HostMidiClock::prepare(double rate) {
    if (!std::isfinite(rate) || rate < 8000.0 || rate > 768000.0)
        return false;
    sampleRate = rate;
    reset();
    return true;
}

void HostMidiClock::reset() {
    expectedPpq = 0.0;
    nextClockTick = 0;
    transportKnown = false;
    wasPlaying = false;
}

std::size_t HostMidiClock::generate(bool positionValid, bool playing,
                                    double bpm, double ppqPosition,
                                    int samples, MidiEvent *events,
                                    std::size_t capacity) {
    if (!positionValid || !events || !capacity || samples <= 0 ||
        !std::isfinite(bpm) || bpm <= 0.0 || !std::isfinite(ppqPosition) ||
        sampleRate <= 0.0)
        return 0;

    std::size_t count = 0;
    const double ppqPerSample = bpm / (60.0 * sampleRate);
    const double endPpq = ppqPosition + ppqPerSample * samples;
    const double continuityTolerance = std::max(1.0e-7, ppqPerSample * 2.0);
    const bool discontinuity = transportKnown && wasPlaying && playing &&
        std::abs(ppqPosition - expectedPpq) > continuityTolerance;

    auto append = [&](std::uint8_t status, int sampleOffset) {
        if (count < capacity)
            events[count++] = {sampleOffset, status, 0, 0};
    };

    if (transportKnown && wasPlaying && (!playing || discontinuity))
        append(0xfc, 0);
    if (playing && (!transportKnown || !wasPlaying || discontinuity)) {
        const bool continueFromStop = transportKnown && !wasPlaying &&
            std::abs(ppqPosition - expectedPpq) <= continuityTolerance;
        append(continueFromStop ? 0xfb : 0xfa, 0);
        if (!continueFromStop)
            nextClockTick = static_cast<std::int64_t>(
                std::ceil(ppqPosition * 24.0 - 1.0e-9));
    }

    if (playing) {
        for (;;) {
            const double tickPpq =
                static_cast<double>(nextClockTick) / 24.0;
            const double exactOffset =
                (tickPpq - ppqPosition) / ppqPerSample;
            const int offset = static_cast<int>(
                std::floor(exactOffset + 0.5));
            if (offset >= samples) break;
            if (count == capacity) break;
            append(0xf8, std::max(0, offset));
            ++nextClockTick;
        }
    }

    transportKnown = true;
    wasPlaying = playing;
    expectedPpq = playing ? endPpq : ppqPosition;
    return count;
}

bool EmulatorBridge::prepare(double sampleRate) {
    if (!clock.prepare(sampleRate)) return false;
    controlRead.store(0, std::memory_order_relaxed);
    controlWrite.store(0, std::memory_order_relaxed);
    controlDrops.store(0, std::memory_order_relaxed);
    publishedCycles.store(0, std::memory_order_relaxed);
    sysExRead.store(0, std::memory_order_relaxed);
    sysExWrite.store(0, std::memory_order_relaxed);
    hasPendingControl = false;
    nextPanelTransitionCycle = 0;
    outputSysExSize = 0;
    outputInSysEx = false;
    sink.prepare(sampleRate);
    return true;
}

bool EmulatorBridge::resetTimeline() {
    const auto sampleRate = clock.rate();
    if (!sampleRate || !clock.prepare(static_cast<double>(sampleRate)))
        return false;
    controlRead.store(0, std::memory_order_relaxed);
    controlWrite.store(0, std::memory_order_relaxed);
    controlDrops.store(0, std::memory_order_relaxed);
    publishedCycles.store(0, std::memory_order_relaxed);
    sysExRead.store(0, std::memory_order_relaxed);
    sysExWrite.store(0, std::memory_order_relaxed);
    hasPendingControl = false;
    nextPanelTransitionCycle = 0;
    outputSysExSize = 0;
    outputInSysEx = false;
    return true;
}

bool EmulatorBridge::enqueue(ControlEvent event) {
    const auto write = controlWrite.load(std::memory_order_relaxed);
    const auto next = (write + 1) % controlQueueCapacity;
    if (next == controlRead.load(std::memory_order_acquire)) {
        controlDrops.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    controls[write] = event;
    controlWrite.store(next, std::memory_order_release);
    return true;
}

bool EmulatorBridge::dequeue(ControlEvent &event) {
    const auto read = controlRead.load(std::memory_order_relaxed);
    if (read == controlWrite.load(std::memory_order_acquire)) return false;
    event = controls[read];
    controlRead.store((read + 1) % controlQueueCapacity,
                      std::memory_order_release);
    return true;
}

bool EmulatorBridge::enqueuePanelTransition(std::uint8_t rawMatrixCode,
                                            bool pressed) {
    if (rawMatrixCode > 0x7f) return false;
    return enqueue({ControlType::panel, rawMatrixCode,
                    static_cast<std::uint16_t>(pressed ? 1 : 0)});
}

bool EmulatorBridge::enqueueKeyboardTransition(std::uint8_t note,
                                               std::uint8_t velocity,
                                               bool pressed) {
    if (note < 36 || note > 96 || (pressed && (velocity < 1 || velocity > 127)))
        return false;
    return enqueue({ControlType::keyboard, note,
                    static_cast<std::uint16_t>(pressed ? velocity : 0)});
}

bool EmulatorBridge::enqueueAnalog(unsigned int channel, std::uint16_t value) {
    if (channel >= 8) return false;
    return enqueue({ControlType::analog, static_cast<std::uint8_t>(channel),
                    value});
}

bool EmulatorBridge::enqueueSysEx(const std::uint8_t *bytes,
                                  std::size_t size) {
    if (!bytes || size < 2 || size > sysExMessageCapacity ||
        bytes[0] != 0xf0 || bytes[size - 1] != 0xf7)
        return false;
    const auto write = sysExWrite.load(std::memory_order_relaxed);
    const auto next = (write + 1) % sysExQueueCapacity;
    if (next == sysExRead.load(std::memory_order_acquire)) return false;
    auto &message = queuedSysEx[write];
    std::copy_n(bytes, size, message.bytes.begin());
    message.size = size;
    sysExWrite.store(next, std::memory_order_release);
    return true;
}

bool EmulatorBridge::dequeueSysEx(QueuedSysEx &message) {
    const auto read = sysExRead.load(std::memory_order_relaxed);
    if (read == sysExWrite.load(std::memory_order_acquire)) return false;
    message = queuedSysEx[read];
    sysExRead.store((read + 1) % sysExQueueCapacity,
                    std::memory_order_release);
    return true;
}

void EmulatorBridge::dispatchControls(std::uint64_t cycle) {
    QueuedSysEx sysEx;
    while (dequeueSysEx(sysEx))
        sink.midiBytes(sysEx.bytes.data(), sysEx.size, cycle);

    ControlEvent event;
    for (;;) {
        if (hasPendingControl) {
            event = pendingControl;
        } else if (!dequeue(event)) {
            return;
        }
        if (event.type == ControlType::panel) {
            if (cycle < nextPanelTransitionCycle) {
                pendingControl = event;
                hasPendingControl = true;
                return;
            }
            hasPendingControl = false;
            sink.panelByte(static_cast<std::uint8_t>(event.first |
                              (event.second ? 0x80 : 0x00)), cycle);
            sink.panelByte(0x00, cycle);
            nextPanelTransitionCycle = cycle + panelTransitionSpacingCycles;
        } else if (event.type == ControlType::analog) {
            hasPendingControl = false;
            sink.analog(event.first, event.second, cycle);
        } else {
            hasPendingControl = false;
            sink.keyboard(event.first,
                          static_cast<std::uint8_t>(event.second),
                          event.second != 0, cycle);
        }
    }
}

void EmulatorBridge::dispatchMidi(const MidiEvent &event, std::uint64_t cycle) {
    if (event.bytes && event.size) {
        sink.midiBytes(event.bytes, event.size, cycle);
        return;
    }
    const auto type = static_cast<std::uint8_t>(event.status & 0xf0);
    if (event.status >= 0xf8 ||
        type == 0x80 || type == 0x90 || type == 0xa0 || type == 0xb0 ||
        type == 0xc0 || type == 0xd0 || type == 0xe0)
        sink.midi(event.status, event.data1, event.data2, cycle);
}

void EmulatorBridge::collectMidiOutput(int sampleOffset,
                                       SysExOutputEvent *output,
                                       std::size_t capacity,
                                       std::size_t &count) {
    std::uint8_t bytes[64];
    for (;;) {
        const auto byteCount = sink.drainMidiOutput(bytes, sizeof(bytes));
        for (std::size_t index = 0; index < byteCount; ++index) {
            const auto value = bytes[index];
            if (value >= 0xf8 && value != 0xf7) continue;
            if (value == 0xf0) {
                outputInSysEx = true;
                outputSysExSize = 0;
            }
            if (!outputInSysEx) continue;
            if (outputSysExSize == outputSysEx.size()) {
                outputInSysEx = false;
                outputSysExSize = 0;
                continue;
            }
            outputSysEx[outputSysExSize++] = value;
            if (value != 0xf7) continue;
            outputInSysEx = false;
            if (output && count < capacity) {
                auto &event = output[count++];
                event.sampleOffset = sampleOffset;
                event.size = outputSysExSize;
                std::copy_n(outputSysEx.begin(), outputSysExSize,
                            event.bytes.begin());
            }
            outputSysExSize = 0;
        }
        if (byteCount < sizeof(bytes)) break;
    }
}

void EmulatorBridge::process(const float *inputLeft, const float *inputRight,
                             float *const *outputChannels,
                             std::size_t outputChannelCount, int samples,
                             const MidiEvent *midiEvents,
                             std::size_t midiEventCount,
                             SysExOutputEvent *sysExOutput,
                             std::size_t sysExOutputCapacity,
                             std::size_t *sysExOutputCount) {
    std::size_t outputCount = 0;
    if (sysExOutputCount) *sysExOutputCount = 0;
    if (samples <= 0 || !outputChannels || outputChannelCount < 2 ||
        !outputChannels[0] || !outputChannels[1]) return;
    if (!sink.beginBlock()) {
        for (std::size_t channel = 0; channel < outputChannelCount; ++channel)
            if (outputChannels[channel])
                std::fill_n(outputChannels[channel], samples, 0.0f);
        return;
    }
    dispatchControls(clock.cycles());
    std::size_t midiIndex = 0;
    for (int sample = 0; sample < samples; ++sample) {
        if ((sample & 63) == 0) dispatchControls(clock.cycles());
        while (midiIndex < midiEventCount &&
               midiEvents[midiIndex].sampleOffset <= sample) {
            dispatchMidi(midiEvents[midiIndex], clock.cycles());
            ++midiIndex;
        }
        const float inLeft = inputLeft ? inputLeft[sample] : 0.0f;
        const float inRight = inputRight ? inputRight[sample] : inLeft;
        sink.samplingInput(inLeft, inRight, clock.cycles());
        const auto cycle = clock.advanceOneSample();
        sink.runUntil(cycle);
        collectMidiOutput(sample, sysExOutput, sysExOutputCapacity,
                          outputCount);
        std::array<float, 8> frame{};
        sink.audioOutput(frame.data(), frame.size(), cycle);
        const auto channels = std::min(outputChannelCount, frame.size());
        for (std::size_t channel = 0; channel < channels; ++channel)
            if (outputChannels[channel])
                outputChannels[channel][sample] = frame[channel];
    }
    while (midiIndex < midiEventCount) {
        dispatchMidi(midiEvents[midiIndex], clock.cycles());
        ++midiIndex;
    }
    collectMidiOutput(samples - 1, sysExOutput, sysExOutputCapacity,
                      outputCount);
    publishedCycles.store(clock.cycles(), std::memory_order_relaxed);
    sink.endBlock();
    if (sysExOutputCount) *sysExOutputCount = outputCount;
}

} // namespace eps16::vst3
