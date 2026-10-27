/* Verify durable UART state without requiring copyrighted firmware files. */
#include "ProbeMachine.c"

static void *legacy_v7(const void *data, size_t size, size_t *legacy_size) {
    const size_t tail = sizeof(Es5505Core) + sizeof(KpcDevice) + m68k_context_size();
    const size_t offset = size - tail - sizeof(duart_rx_a_enabled);
    *legacy_size = size - sizeof(duart_rx_a_enabled);
    uint8_t *result = malloc(*legacy_size);
    if (!result) return NULL;
    memcpy(result, data, offset);
    memcpy(result + offset, (const uint8_t *)data + offset + sizeof(duart_rx_a_enabled), tail);
    PluginSnapshotHeader *header = (PluginSnapshotHeader *)result;
    header->version = 7;
    header->total_size = *legacy_size;
    header->checksum = snapshot_checksum(result + sizeof(*header), *legacy_size - sizeof(*header));
    return result;
}

int main(void) {
    Eps16ProbeMachine *machine = eps16_probe_machine_create();
    if (!machine || !eps16_probe_machine_begin(machine)) return 1;
    m68k_init();
    m68k_set_cpu_type(M68K_CPU_TYPE_68000);
    es5505_core_init(&es5505, NULL, NULL);
    es5510_core_init(&es5510);
    plugin_initialized = 1;
    char error[256] = {0};
    for (unsigned int enabled = 0; enabled <= 1; ++enabled) {
        duart_write(DUART_BASE + 4, enabled ? 0x01 : 0x02);
        duart_write(DUART_BASE + 10, enabled ? 0x02 : 0x00);
        duart_write(DUART_BASE + 4, 0x08); /* TX disable must retain RX. */
        const size_t size = eps16_probe_machine_state_size();
        void *snapshot = malloc(size);
        if (!snapshot || !eps16_probe_machine_save_state(snapshot, size)) return 2;
        for (unsigned int legacy = 0; legacy <= 1; ++legacy) {
            size_t bytes = size;
            void *data = legacy ? legacy_v7(snapshot, size, &bytes) : snapshot;
            if (!data) return 3;
            duart_rx_a_enabled = !enabled;
            if (!eps16_probe_machine_load_state(data, bytes, error, sizeof(error))) {
                fprintf(stderr, "restore failed: %s\n", error); return 4;
            }
            eps16_probe_machine_midi(0x90, 60, 100);
            midi_service_wire(bus_cycle_now() + MIDI_BYTE_CYCLES * 4);
            if (duart_rx_a_enabled != (int)enabled || midi_rx_count != (enabled ? 3 : 0)) {
                fprintf(stderr, "v%u RX=%d bytes=%zu expected=%u\n", legacy ? 7 : 8,
                        duart_rx_a_enabled, midi_rx_count, enabled); return 5;
            }
            printf("v%u receiver=%u MIDI bytes=%zu OK\n", legacy ? 7 : 8, enabled, midi_rx_count);
            if (legacy) free(data);
        }
        free(snapshot);
    }
    eps16_probe_machine_end(machine);
    eps16_probe_machine_destroy(machine);
    return 0;
}
