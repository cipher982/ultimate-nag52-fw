#ifndef BOOT_INFO_H
#define BOOT_INFO_H

#include <stdint.h>

/**
 * Why the TCU last restarted.
 *
 * This firmware ran for its whole life without recording that. Five
 * transmission-controller reboots on entry to Park were reconstructed from the
 * CAN wire alone -- a rolling counter driven to zero and payloads reverting to
 * FF signal-invalid fill. That evidence proves the CAN object was rebuilt. It
 * says nothing about *why*, and the candidates need opposite fixes:
 *
 *   panic / assert      -> a software defect, fix the code
 *   interrupt watchdog  -> something disabled interrupts too long
 *   task watchdog       -> a task starved
 *   brownout            -> an electrical problem, not a software one
 *   software reset      -> deliberate, and we should know who asked
 *
 * The bootloader prints this on UART at every boot, but that is only useful
 * with somebody sitting in the vehicle holding a cable. Latching it in RAM and
 * serving it over KWP means the next restart explains itself on the next
 * connection instead of after another week of driving.
 *
 * Deliberately no NVS write. Persisting a boot counter would add a flash write
 * to the boot path, and unguarded flash writes are the very thing under
 * investigation.
 */
namespace BootInfo {
    /** Latch esp_reset_reason(). Call once, first thing in app_main. */
    void record_reset_reason(void);

    /** The latched value, as esp_reset_reason_t. */
    uint8_t get_reset_reason(void);
}

/** Payload for RLI_BOOT_INFO. Little-endian, packed, append-only. */
typedef struct {
    /** Bump when fields are added so a reader can tell what it is looking at. */
    uint8_t schema;
    /** esp_reset_reason_t latched at boot. */
    uint8_t reset_reason;
    uint16_t reserved;
    /** Milliseconds since boot, so a reader can tell a fresh restart from an old one. */
    uint32_t uptime_ms;
} __attribute__ ((packed)) BOOT_INFO;

#endif
