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
		LOG_INF("RX SET_MAG_BIAS [%.3f %.3f %.3f]",
		        (double)bias[0], (double)bias[1], (double)bias[2]);
		rft_mag_save_bias(bias);
		break;
	}
	case RFT_CMD_CLEAR_MAG_BIAS:
		LOG_INF("RX CLEAR_MAG_BIAS");
		rft_mag_clear_bias();
		break;
	case RFT_CMD_MAG_RECAL:
		LOG_INF("RX MAG_RECAL");
		rft_mag_cal_reset();
		break;
	default:
		LOG_WRN("Unknown cmd type %u", type);
		break;
	}
}

void rft_mag_cmd_handle_rx(const uint8_t slot[10], uint8_t my_tracker_id)
{
	uint8_t target = slot[0];
	if (target == RFT_CMD_NO_TARGET || target != my_tracker_id) return;

	uint8_t type = slot[1];
	if (type == RFT_CMD_NONE) return;

	/* Verify XOR checksum: bytes [0..8] XOR == byte [9] */
	uint8_t xor = 0;
	for (int i = 0; i < 9; i++) xor ^= slot[i];
	if (xor != slot[9]) {
		LOG_WRN("cmd checksum mismatch");
		return;
	}

	/* Capture and defer to work queue (ISR-safe). If a previous command is
	 * still pending and unexecuted, drop the new one rather than racing. */
	if (pending.valid) return;
	pending.type = type;
	memcpy(pending.data, &slot[2], 6);
	pending.valid = true;
	k_work_submit(&rft_mag_cmd_work);
}
