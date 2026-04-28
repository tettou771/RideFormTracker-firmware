/*
	RFT mag-cal command receiver (tracker side).
	See rft_mag_cmd.h for protocol description.
*/
#include "rft_mag_cmd.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(rft_mag_cmd, LOG_LEVEL_INF);

/* Forward declarations of the existing mag-cal helpers in sensor.c. Keeping
 * the linkage explicit (rather than via header) because we don't want to
 * pull in all of sensor.h here. */
extern void rft_mag_save_bias(const float bias[3]);
extern void rft_mag_clear_bias(void);
extern void rft_mag_cal_reset(void);

/* Packet 8 streaming flag, read by sensor.c to gate transmission of the
 * raw_mag + bias announcement packet. Default off — tracker stays quiet
 * unless the PC explicitly asks for the data (e.g., user pressed Reset). */
volatile bool rft_mag_stream_enabled = false;

/* Pending command captured in ISR, executed by work-queue handler. */
static struct {
	uint8_t type;
	uint8_t data[6];
	bool valid;
} pending;

static void rft_mag_cmd_work_handler(struct k_work *work);
static K_WORK_DEFINE(rft_mag_cmd_work, rft_mag_cmd_work_handler);

static void rft_mag_cmd_work_handler(struct k_work *work)
{
	if (!pending.valid) return;
	uint8_t type = pending.type;
	uint8_t buf[6];
	memcpy(buf, pending.data, 6);
	pending.valid = false;

	switch (type) {
	case RFT_CMD_SET_MAG_BIAS: {
		int16_t qx, qy, qz;
		memcpy(&qx, &buf[0], 2);
		memcpy(&qy, &buf[2], 2);
		memcpy(&qz, &buf[4], 2);
		float bias[3] = {
			qx * (1.0f / 2048.0f),
			qy * (1.0f / 2048.0f),
			qz * (1.0f / 2048.0f)
		};
		printk("RFT_CMD: RX SET_MAG_BIAS [%.3f %.3f %.3f]\n",
		       (double)bias[0], (double)bias[1], (double)bias[2]);
		rft_mag_save_bias(bias);
		break;
	}
	case RFT_CMD_CLEAR_MAG_BIAS:
		printk("RFT_CMD: RX CLEAR_MAG_BIAS\n");
		rft_mag_clear_bias();
		break;
	case RFT_CMD_MAG_RECAL:
		printk("RFT_CMD: RX MAG_RECAL\n");
		rft_mag_cal_reset();
		break;
	default:
		printk("RFT_CMD: unknown type %u\n", type);
		break;
	}
}

void rft_mag_cmd_handle_rx(const uint8_t slot[10], uint8_t my_tracker_id)
{
	/* Verify XOR checksum first. If it mismatches we can't trust any of
	 * the bytes, including the flag byte. */
	uint8_t xor = 0;
	for (int i = 0; i < 9; i++) xor ^= slot[i];
	if (xor != slot[9]) {
		/* Quiet — not LOG_WRN — bad packets are normal during pairing
		 * churn and would otherwise spam the console. */
		return;
	}

	/* (1) Global flags — applied to every tracker on every sync. */
	uint8_t flags = slot[0];
	bool want_stream = (flags & RFT_FLAG_STREAM_RAW_MAG) != 0;
	if (want_stream != rft_mag_stream_enabled) {
		rft_mag_stream_enabled = want_stream;
		printk("RFT_FLAG: STREAM_RAW_MAG=%d\n", (int)want_stream);
	}

	/* (2) Diagnostic: count every sync RX so we can confirm the path is alive
	 * even when no command targets us. Prints once a second. */
	static volatile uint32_t sync_rx_count = 0;
	static int64_t last_log_ms = 0;
	sync_rx_count++;
	int64_t now_ms = k_uptime_get();
	if (now_ms - last_log_ms > 1000) {
		last_log_ms = now_ms;
		printk("RFT_CMD: sync_rx/s=%u flags=0x%02x target=%u (me=%u) type=%u\n",
		       sync_rx_count, slot[0], slot[1], my_tracker_id, slot[2]);
		sync_rx_count = 0;
	}

	/* (3) Targeted command. */
	uint8_t target = slot[1];
	if (target == RFT_CMD_NO_TARGET || target != my_tracker_id) return;

	uint8_t type = slot[2];
	if (type == RFT_CMD_NONE) return;

	/* Capture and defer to work queue (ISR-safe). If a previous command is
	 * still pending and unexecuted, drop the new one rather than racing. */
	if (pending.valid) return;
	pending.type = type;
	memcpy(pending.data, &slot[3], 6);
	pending.valid = true;
	k_work_submit(&rft_mag_cmd_work);
}
