/* Exercise the actual machine renderer and audio queue with a synthetic voice
   and DSP program. Including the private adapter keeps test-only register setup
   out of the public plug-in API. No Ensoniq ROM or OS files are required. */
#include "ProbeMachine.c"

static uint16_t routing_test_sample(void *context, unsigned int bank,
                                    uint32_t address) {
    (void)context; (void)bank; (void)address;
    return 12000;
}

static uint64_t routing_instruction(uint8_t source, uint8_t destination,
                                     uint8_t operation) {
    return (UINT64_C(0xff) << 40) | (UINT64_C(0xff) << 32) |
           ((uint64_t)source << 24) | ((uint64_t)destination << 16) |
           ((uint64_t)operation << 12);
}

int main(void) {
    /* EPS assignments use raw 0=Bus1, 2=Bus2, 3=Bus3, 1=Aux1. */
    const unsigned int assignments[] = {0, 2, 3, 1};
    for (unsigned int route = 0; route < 4; ++route) {
        Eps16ProbeMachine *machine = eps16_probe_machine_create();
        if (!machine || !eps16_probe_machine_begin(machine)) return 1;
        es5505_core_init(&es5505, routing_test_sample, NULL);
        es5505.active_voice = 20;
        Es5505Voice *voice = &es5505.voices[0];
        voice->control = (uint16_t)((assignments[route] << 8) |
                                  (3 << 10) | ES5505_LOOP_ENABLE);
        voice->end = 1024U << 11;
        voice->frequency = 1U << 11;
        voice->k1 = voice->k2 = 0xfff0;
        voice->left_volume = voice->right_volume = 255;
        analog_values[5] = 1023U << 6;
        duart_output = 0x04; /* Board input mux selects Bus1. */
        es5510_core_init(&es5510);
        /* Sum all three FX input pairs (SER0, SER2, SER3) into SER1.
           Aux1 must never enter this sum. */
        es5510.instruction[0] = routing_instruction(0xeb, 0, 9);
        es5510.instruction[1] = routing_instruction(0xea, 1, 9);
        es5510.instruction[2] = routing_instruction(0xef, 0, 0);
        es5510.instruction[3] = routing_instruction(0xee, 1, 0);
        es5510.instruction[4] = routing_instruction(0xf1, 0, 0);
        es5510.instruction[5] = routing_instruction(0xf0, 1, 0);
        es5510.instruction[6] = routing_instruction(0, 0xed, 9);
        es5510.instruction[7] = routing_instruction(1, 0xec, 9);
        es5510.instruction[8] = routing_instruction(0xff, 0xff, 15);
        es5510_core_set_host_serial(&es5510, 0x08);
        es5510_core_set_halted(&es5510, 0);
        plugin_render_audio(336U * 64, 336U * 64);
        Eps16ProbeAudioFrame frames[128];
        const size_t count = eps16_probe_machine_drain_audio(frames, 128);
        float peaks[8] = {0};
        for (size_t frame = 0; frame < count; ++frame)
            for (unsigned int channel = 0; channel < 8; ++channel) {
                const float peak = fabsf(frames[frame].outputs[channel]);
                if (peak > peaks[channel]) peaks[channel] = peak;
            }
        if (!count) return 2;
        for (unsigned int pair = 0; pair < 4; ++pair) {
            const int audible = pair == 0 ? route != 3 : route == pair;
            for (unsigned int side = 0; side < 2; ++side) {
                const float peak = peaks[pair * 2 + side];
                if ((audible && peak < 0.01f) || (!audible && peak != 0.0f)) {
                    fprintf(stderr, "route=%u raw=%u output=%u peak=%f expected=%s\n",
                            route, assignments[route], pair, peak,
                            audible ? "signal" : "silence");
                    return 3;
                }
            }
        }
        printf("OEX route %u/raw %u: Main=%f Bus2=%f Bus3=%f Aux1=%f\n",
               route, assignments[route], peaks[0], peaks[2], peaks[4], peaks[6]);
        eps16_probe_machine_end(machine);
        eps16_probe_machine_destroy(machine);
    }
    return 0;
}
