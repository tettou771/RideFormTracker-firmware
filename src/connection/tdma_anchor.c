/*
    RFT TDMA anchor — tracker side. See tdma_anchor.h.
*/
#include "globals.h"
#include "tdma_anchor.h"
#include "tdma_proto.h"
#include "system/system.h"

#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(tdma_anchor, LOG_LEVEL_INF);

/* Lose anchor if no TIMING ACK arrives within this many ms — fall back
 * to ALOHA discovery. ~5 cycles at 30 Hz is the tolerance. */
#define TDMA_ANCHOR_LOSS_MS         200

/* ALOHA discovery TX period. Slow enough to not contend heavily, fast
 * enough that the receiver picks us up within a few seconds. */
#define TDMA_DISCOVERY_PERIOD_MS    1000

static uint8_t  slot_index       = TDMA_SLOT_UNSET;
static uint32_t cycle_counter_lo = 0;   /* low 16 bits of receiver's counter */

/* Last time we received a TIMING ACK, in tracker's k_uptime_get() ms. */
static int64_t  last_timing_ms   = 0;

/* Slot trigger: the semaphore connection_thread sleeps on. Given by
 * a k_timer in TDMA mode or by an internal k_timer in discovery mode. */
static K_SEM_DEFINE(slot_trigger, 0, 1);

/* HW timer for the per-slot TX. In TDMA mode we (re)arm this for the
 * computed slot time. ALOHA uses a separate periodic timer. */
static void slot_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_sem_give(&slot_trigger);
}
K_TIMER_DEFINE(slot_timer, slot_timer_handler, NULL);

/* ALOHA discovery: periodic 1 Hz kick when there's no TIMING anchor.
 * Started/stopped from tdma_anchor_wait_slot() based on state. */
static void discovery_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_sem_give(&slot_trigger);
}
K_TIMER_DEFINE(discovery_timer, discovery_timer_handler, NULL);
static bool discovery_running = false;

void tdma_anchor_init(void)
{
    uint8_t s = TDMA_SLOT_UNSET;
    sys_read(RFT_TRACKER_SLOT_ID, &s, sizeof(s));
    /* NVS may return 0 for fresh devices (no entry) — treat as unset
     * only if the value is out of range, else trust it. */
    if (s == 0xFF) {
        slot_index = TDMA_SLOT_UNSET;
        LOG_INF("TDMA anchor: slot UNSET (NVS empty)");
    } else if (s >= TDMA_NUM_SLOTS) {
        slot_index = TDMA_SLOT_UNSET;
        LOG_WRN("TDMA anchor: NVS slot=%u out of range, treating as unset", s);
    } else {
        slot_index = s;
        LOG_INF("TDMA anchor: slot=%u loaded from NVS", s);
    }
}

/* Init early so connection_thread (which blocks on tdma_anchor_wait_slot)
 * has correct state before its first iteration. APPLICATION priority is
 * after retained-mem / NVS subsystems are ready. */
static int tdma_anchor_sys_init(void)
{
    tdma_anchor_init();
    return 0;
}
SYS_INIT(tdma_anchor_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

uint8_t tdma_anchor_get_slot(void)
{
    return slot_index;
}

void tdma_anchor_set_slot(uint8_t idx)
{
    if (idx >= TDMA_NUM_SLOTS) {
        LOG_WRN("TDMA anchor: refusing slot=%u (out of range)", idx);
        return;
    }
    slot_index = idx;
    sys_write(RFT_TRACKER_SLOT_ID, NULL, &idx, sizeof(idx));
    LOG_INF("TDMA anchor: slot set to %u (persisted)", idx);
}

bool tdma_anchor_on_ack_rx(const uint8_t *payload, uint8_t len, int64_t rx_uptime_us)
{
    if (len < 12) return false;
    if (payload[0] != PKT_TDMA_BEACON) return false;   /* not a TIMING ACK */

    /* Verify checksum. Cheap XOR (matches receiver tdma_pack_timing_ack). */
    uint8_t x = 0;
    for (int i = 0; i < 11; i++) x ^= payload[i];
    if (x != payload[11]) {
        LOG_DBG("TDMA TIMING ACK: bad checksum");
        return true;  /* still a TIMING ACK, just rejected — don't fall through to CMD */
    }

    /* Silent until we have a slot — drop the timing on the floor. The
     * ALOHA fallback won't fire either, so we stay quiet. */
    if (slot_index == TDMA_SLOT_UNSET) return true;

    uint32_t us_until_next = (uint32_t)payload[1]
                           | ((uint32_t)payload[2] << 8)
                           | ((uint32_t)payload[3] << 16);
    uint16_t cyc_lo        = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
    uint16_t stride_us     = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
    uint16_t head_us       = (uint16_t)payload[8] * 8u;  /* encoded in 8 µs steps */
    /* payload[9] = num_slots (informational), payload[10] = reserved */

    /* Schedule the next TX. Anchor in tracker's microsecond clock:
     *   next_cycle_start (in tracker time) = rx_uptime_us + us_until_next
     *   my_tx_time                         = above + head + index * stride
     * The receiver's staleness budget (~1 ms from 1 kHz refresh + ~stride
     * from FIFO depth) shows up here as an offset on us_until_next; the
     * tracker can't compensate, so the slot lands a few ms late. Tolerable
     * inside a ~3 ms slot at 30 Hz; tightens as rate increases. */
    int64_t tx_us = rx_uptime_us + (int64_t)us_until_next
                  + (int64_t)head_us
                  + (int64_t)stride_us * (int64_t)slot_index;
    int64_t now_us  = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t wait_us = tx_us - now_us;
    if (wait_us < 0) wait_us = 0;
    if (wait_us > 1000000) wait_us = 1000000;   /* clamp absurd values */

    /* Re-arm (one-shot). The previous unfired timer, if any, is cancelled. */
    k_timer_start(&slot_timer, K_USEC(wait_us), K_NO_WAIT);

    cycle_counter_lo = cyc_lo;
    last_timing_ms   = k_uptime_get();

    /* Stop the ALOHA discovery timer if it was running — we have an anchor now. */
    if (discovery_running) {
        k_timer_stop(&discovery_timer);
        discovery_running = false;
        LOG_INF("TDMA anchor: TRACKING (stride=%u head=%u wait=%lld us)",
                stride_us, head_us, (long long)wait_us);
    }
    return true;
}

uint32_t tdma_anchor_wait_slot(void)
{
    /* SILENT: no slot. Sleep coarsely so the caller doesn't spin. */
    if (slot_index == TDMA_SLOT_UNSET) {
        k_msleep(500);
        return 0;
    }

    /* If we haven't seen a TIMING ACK recently, we're in DISCOVERY mode.
     * Ensure the periodic discovery timer is running and wait on the
     * trigger. */
    int64_t now_ms     = k_uptime_get();
    bool    tracking   = (now_ms - last_timing_ms) < TDMA_ANCHOR_LOSS_MS;
    if (!tracking && !discovery_running) {
        k_timer_start(&discovery_timer, K_MSEC(TDMA_DISCOVERY_PERIOD_MS),
                      K_MSEC(TDMA_DISCOVERY_PERIOD_MS));
        discovery_running = true;
        LOG_INF("TDMA anchor: DISCOVERY (no timing >%dms)", TDMA_ANCHOR_LOSS_MS);
    }

    k_sem_take(&slot_trigger, K_FOREVER);
    return cycle_counter_lo;
}

tdma_anchor_state_t tdma_anchor_get_state(void)
{
    if (slot_index == TDMA_SLOT_UNSET) return TDMA_ANCHOR_SILENT;
    if (k_uptime_get() - last_timing_ms < TDMA_ANCHOR_LOSS_MS) return TDMA_ANCHOR_TRACKING;
    return TDMA_ANCHOR_DISCOVERY;
}
