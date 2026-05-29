/*
	RFT mag-cal command receiver (tracker side).

	The receiver broadcasts a 12-byte sync packet (delivered as the ACK
	payload to each tracker on pipe 1). Two distinct sub-channels:

	1. Global flags (broadcast, replicated to all): byte [2].
	   Currently RFT_FLAG_STREAM_RAW_MAG. State propagates to every paired
	   tracker continuously — no retransmit needed.

	2. Targeted command (single-tracker): bytes [3..11]. ACK FIFO is
	   shared depth-1 broadcast, so the receiver retransmits the same
	   payload many times to defeat random consumption by other trackers.

	Wire format on the sync packet (12 bytes):
	  [0..1]  LED clock (legacy)
	  [2]     global flags (RFT_FLAG_*)
	  [3]     target tracker ID (0xFF = no targeted command)
	  [4]     command type (RFT_CMD_*)
	  [5..10] payload (6 bytes, command-specific)
	  [11]    XOR checksum of [2..10]

	ESB RX runs in interrupt context, so the actual NVS-touching handler runs
	on a Zephyr work queue. Constants must match the receiver-side cmd_queue.h.
*/
#ifndef RFT_MAG_CMD_H
#define RFT_MAG_CMD_H

#include <stdint.h>

/* Global flags (byte [2]) */
#define RFT_FLAG_STREAM_RAW_MAG (1u << 0)

/* Targeted command types (byte [4]) */
#define RFT_CMD_NONE           0
#define RFT_CMD_SET_MAG_BIAS   1  /* payload: 3 × int16 Q11 (Gauss) */
#define RFT_CMD_CLEAR_MAG_BIAS 2
#define RFT_CMD_MAG_RECAL      3
/* RFT_CMD_STREAM_RAW_MAG (=4) was retired; replaced by RFT_FLAG_STREAM_RAW_MAG. */
#define RFT_CMD_SET_MAX_RATE_HZ 5 /* payload[0]: uint8 Hz, 0 = back to firmware
                                   * default. RAM only — power cycle reverts. */
#define RFT_CMD_SET_SLOT_INDEX  6 /* payload[0]: uint8 slot index (0..MAX_SENSORS-1).
                                   * Tracker stores in NVS (RFT_TRACKER_SLOT_ID).
                                   * Sent by the receiver right after pair completes
                                   * or in response to the `broadcast_index` console
                                   * command. See tdma_proto.h / PLAN_tdma.md. */

#define RFT_CMD_NO_TARGET      0xFF

/* Called from the ESB RX interrupt handler with the 10-byte payload tail
 * (sync packet bytes [2..11]). Applies global flags to all trackers and,
 * if target byte matches my_tracker_id, dispatches the targeted command
 * (deferred to a work queue, NVS-safe). Safe to call from ISR. */
void rft_mag_cmd_handle_rx(const uint8_t slot[10], uint8_t my_tracker_id);

#endif
