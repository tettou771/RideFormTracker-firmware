/*
	RFT mag-cal command receiver (tracker side).

	The receiver broadcasts a 12-byte sync packet to all paired trackers every
	~3ms. Bytes 0..1 are the LED clock (existing behaviour). Bytes 2..11 carry
	an optional command targeting one specific tracker:
	  [2]   target tracker ID (0xFF = no command)
	  [3]   command type (RFT_CMD_*)
	  [4..9] payload (6 bytes, command-specific)
	  [10]  reserved
	  [11]  XOR checksum of bytes [2..10]

	ESB RX runs in interrupt context, so the actual NVS-touching handler runs
	on a Zephyr work queue. Constants must match the receiver-side cmd_queue.h.
*/
#ifndef RFT_MAG_CMD_H
#define RFT_MAG_CMD_H

#include <stdint.h>

#define RFT_CMD_NONE           0
#define RFT_CMD_SET_MAG_BIAS   1  /* payload: 3 × int16 Q11 (Gauss) */
#define RFT_CMD_CLEAR_MAG_BIAS 2
#define RFT_CMD_MAG_RECAL      3

#define RFT_CMD_NO_TARGET      0xFF

/* Called from the ESB RX interrupt handler with the 10-byte command slot
 * (sync packet bytes [2..11]). Verifies target ID + checksum, defers actual
 * action to a work queue. Safe to call from ISR. */
void rft_mag_cmd_handle_rx(const uint8_t slot[10], uint8_t my_tracker_id);

#endif
