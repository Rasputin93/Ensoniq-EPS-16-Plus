#include "ProbeMachine.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int load_snapshot(const char *path, char *error, size_t error_size) {
    FILE *file = fopen(path, "rb");
    if (!file || fseek(file, 0, SEEK_END)) return 0;
    const long size = ftell(file);
    if (size <= 0 || fseek(file, 0, SEEK_SET)) return 0;
    void *data = malloc((size_t)size);
    const int loaded = data && fread(data, 1, (size_t)size, file) == (size_t)size &&
        eps16_probe_machine_load_state(data, (size_t)size, error, error_size);
    free(data);
    fclose(file);
    return loaded;
}

static float render_activity(uint64_t cycles) {
    const uint64_t start = eps16_probe_machine_cycles();
    eps16_probe_machine_reset_audio_peaks();
    while (eps16_probe_machine_cycles() - start < cycles) {
        eps16_probe_machine_run_until(eps16_probe_machine_cycles() + 100000);
    }
    float activity = eps16_probe_machine_output_peak();
    for (unsigned int bus = 0; bus < 4; ++bus) {
        const float peak = eps16_probe_machine_es5505_bus_peak(bus);
        if (peak > activity) activity = peak;
    }
    const float effect_peak = eps16_probe_machine_es5510_return_peak();
    return effect_peak > activity ? effect_peak : activity;
}

static Eps16ProbeAudioDebugState debug_state(void) {
    Eps16ProbeAudioDebugState result;
    eps16_probe_machine_audio_debug_state(&result);
    return result;
}

int main(int argc, char **argv) {
    if (argc != 5) {
        fprintf(stderr, "usage: %s COMBINED_ROM KPC_ROM OS_DISK SNAPSHOT\n", argv[0]);
        return 2;
    }
    Eps16ProbeMachine *machine = eps16_probe_machine_create();
    char error[256];
    if (!machine || !eps16_probe_machine_begin(machine) ||
        !eps16_probe_machine_initialize(argv[1], argv[2], argv[3],
                                        error, sizeof(error)) ||
        !load_snapshot(argv[4], error, sizeof(error))) {
        fprintf(stderr, "machine setup failed: %s\n", error);
        return 1;
    }

    /* The GUI sends press and release as separate KPC panel transitions.
       Keep their actual 500,000-cycle spacing so the regression exercises
       the same asynchronous path as a mouse click in the plug-in. */
    /* Multi presets may select any base channel.  Excite every incoming
       channel once so the test remains about the panel event, not a preset's
       MIDI-channel assignment. */
    for (unsigned int channel = 0; channel < 16; ++channel)
        eps16_probe_machine_midi((uint8_t)(0x90 | channel), 60, 100);
    const float before = render_activity(3000000);
    const float baseline = render_activity(8000000);
    const Eps16ProbeAudioDebugState baseline_state = debug_state();

    /* Rewind to the exact same loaded-Multi state for the panel branch. */
    if (!load_snapshot(argv[4], error, sizeof(error))) {
        fprintf(stderr, "snapshot reload failed: %s\n", error);
        return 1;
    }
    for (unsigned int channel = 0; channel < 16; ++channel)
        eps16_probe_machine_midi((uint8_t)(0x90 | channel), 60, 100);
    (void)render_activity(3000000);
    eps16_probe_machine_panel_byte(0x82);
    eps16_probe_machine_panel_byte(0x00);
    eps16_probe_machine_run_until(eps16_probe_machine_cycles() + 500000);
    eps16_probe_machine_panel_byte(0x02);
    eps16_probe_machine_panel_byte(0x00);
    const float after = render_activity(8000000);
    const Eps16ProbeAudioDebugState after_state = debug_state();
    for (unsigned int channel = 0; channel < 16; ++channel)
        eps16_probe_machine_midi((uint8_t)(0x80 | channel), 60, 0);

    printf("track_select_audio before=%f baseline=%f after=%f es5505=%f/%f/%f/%f "
           "es5510=%f state=%u/%d/%d/%zu->%u/%d/%d/%zu\n", before, baseline, after,
           eps16_probe_machine_es5505_bus_peak(0),
           eps16_probe_machine_es5505_bus_peak(1),
           eps16_probe_machine_es5505_bus_peak(2),
           eps16_probe_machine_es5505_bus_peak(3),
           eps16_probe_machine_es5510_return_peak(),
           baseline_state.active_voices, baseline_state.es5510_halted,
           baseline_state.host_upload_active,
           baseline_state.pending_panel_transitions,
           after_state.active_voices, after_state.es5510_halted,
           after_state.host_upload_active,
           after_state.pending_panel_transitions);
    eps16_probe_machine_end(machine);
    eps16_probe_machine_destroy(machine);
    return before > 0.00001f && after >= baseline * 0.1f ? 0 : 1;
}
