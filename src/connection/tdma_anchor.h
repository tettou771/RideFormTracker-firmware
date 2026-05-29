/*
    RFT TDMA anchor — tracker side.

    Owns the tracker's slot index (NVS-persisted under RFT_TRACKER_SLOT_ID)
    and the slot-tick scheduler driven by TIMING ACKs from the receiver.

    Wire format: every ACK the tracker receives is either a CMD ACK
    (legacy LED clock + rft_cmd payload) or a TIMING ACK (data[0]=0xFB).
    See firmware-receiver/src/connection/tdma.h for the receiver's
    pack format; this anchor is the consumer.

    Connection thread cooperation:
      - At boot, tdma_anchor_init() loads slot from NVS.
        slot == 0xFF (unset) → tracker stays silent until SET_SLOT_INDEX.
        slot set            → tracker enters ALOHA discovery (slow 1 Hz
                              TX) until the first TIMING ACK arrives.
      - On TIMING ACK (called from the ESB ISR):
          tdma_anchor_on_timing_ack() arms a HW timer for the tracker's
          next slot.
      - connection_thread() calls tdma_anchor_wait_slot() at the top of
        each iteration. Blocks until either the HW timer fires (TDMA
        active) OR the 1 Hz ALOHA tick (discovery) — returns the cycle
        counter so callers can rotate housekeeping packets.
*/
#ifndef RFT_TDMA_ANCHOR_H
#define RFT_TDMA_ANCHOR_H

#include <stdint.h>
#include <stdbool.h>

/* Tracker's slot is unset/invalid. */
#define TDMA_SLOT_UNSET 0xFF

/* One-shot init. Loads slot from NVS. */
void tdma_anchor_init(void);

/* Get / set the tracker's slot index. set() persists to NVS. */
uint8_t tdma_anchor_get_slot(void);
void    tdma_anchor_set_slot(uint8_t idx);

/* Called from the ESB ISR on every RX (any ACK payload). Discriminates
 * on payload[0]==0xFB and, if TIMING, schedules the tracker's next TX.
 * Returns true if this was a TIMING ACK (caller should skip CMD parsing). */
bool tdma_anchor_on_ack_rx(const uint8_t *payload, uint8_t len, int64_t rx_uptime_us);

/* Block the calling thread until the next slot tick (TDMA mode) or
 * 1 s ALOHA tick (discovery). Returns the cycle counter as of this
 * tick (low 16 bits; suitable for `cycle % period == my_index` rotation).
 * If slot is unset returns 0 immediately AFTER a long sleep (silent
 * mode — caller should NOT TX). */
uint32_t tdma_anchor_wait_slot(void);

/* States: silent (slot unset), discovery (slot set, no anchor yet),
 * tracking (anchored). For LED / diag logging. */
typedef enum {
    TDMA_ANCHOR_SILENT    = 0,  /* don't TX */
    TDMA_ANCHOR_DISCOVERY = 1,  /* slow ALOHA TX */
    TDMA_ANCHOR_TRACKING  = 2,  /* anchored, TX in slot */
} tdma_anchor_state_t;
tdma_anchor_state_t tdma_anchor_get_state(void);

#endif
