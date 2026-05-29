/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "util.h"
#include "esb.h"
#include "build_defines.h"
#include "hid.h"
#include "system/battery_tracker.h"
#include "tdma_anchor.h"
#include "tdma_proto.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status, tracker_button;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static uint8_t data_buffer[21] = {0};
static int64_t last_data_time = 0;
static uint8_t packet_sequence = 0;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

static void connection_thread(void);
K_THREAD_DEFINE(connection_thread_id, 512, connection_thread, NULL, NULL, NULL, CONNECTION_THREAD_PRIORITY, K_FP_REGS, 0);

K_MUTEX_DEFINE(data_buffer_mutex);

void connection_clocks_request_start(void)
{
	clocks_request_start(0);
}

void connection_clocks_request_start_delay_us(uint32_t delay_us)
{
	clocks_request_start(delay_us);
}

void connection_clocks_request_stop(void)
{
	clocks_stop();
}

void connection_clocks_request_stop_delay_us(uint32_t delay_us)
{
	clocks_request_stop(delay_us);
}

uint8_t connection_get_id(void)
{
	return tracker_id;
}

void connection_set_id(uint8_t id)
{
	tracker_id = id;
}

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	// not using get_server_constant_mag_id, does not exist in server enums
	if (mag < 0)
		mag_id = SVR_MAG_STATUS_NOT_SUPPORTED;
	else if CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_MAG)
		mag_id = SVR_MAG_STATUS_ENABLED;
	else
		mag_id = SVR_MAG_STATUS_DISABLED;
}

static int64_t quat_update_time = 0;
static int64_t last_quat_time = 0;
static bool send_precise_quat;

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
	send_precise_quat = q_epsilon(q, sensor_q, 0.005);
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	quat_update_time = k_uptime_get();
}

static int64_t mag_update_time = 0;
static int64_t last_mag_time = 0;

void connection_update_sensor_mag(float *m)
{
	memcpy(sensor_m, m, sizeof(sensor_m));
	mag_update_time = k_uptime_get();
}

// RFT: raw mag + tracker-side bias for PC-side visualisation. Sent in the
// new low-rate packet 8 (~10Hz, matching mag sampling). Real-time-changing
// values like the disturbance angle d stay in packet 4 at full rate.
static float sensor_raw_mag[3] = {0, 0, 0};
static float sensor_mag_bias[3] = {0, 0, 0};
static uint8_t sensor_mag_cal_done = 0;

void connection_update_sensor_raw_mag(const float m[3])
{
	memcpy(sensor_raw_mag, m, sizeof(sensor_raw_mag));
}

void connection_update_sensor_mag_bias(const float bias[3], bool cal_done)
{
	memcpy(sensor_mag_bias, bias, sizeof(sensor_mag_bias));
	sensor_mag_cal_done = cal_done ? 1 : 0;
}

// VQF heading disturbance angle (rad, ±π). Sent in packet 4.
static float sensor_diag_dis_angle = 0.0f;

void connection_update_sensor_diag(float disAngle, float biasMag, float sphereR)
{
	sensor_diag_dis_angle = disAngle;
	(void)biasMag;   /* now part of packet 8 */
	(void)sphereR;   /* PC computes sphere fit, no longer transmitted */
}

// RFT diag packet 9: vectors that go into VQF's heading disagreement
// computation. Lets the host visualise mag in both device and earth frame
// (post-VQF-rotation) to localize coord/handedness issues.
//   d == atan2(magEarth.x, magEarth.y) - delta
static float sensor_mag_device[3] = {0, 0, 0};  // post-alignment, pre-VQF
static float sensor_mag_earth[3]  = {0, 0, 0};  // gravity-aligned by gyro+accel quat
static float sensor_vqf_delta     = 0.0f;       // VQF heading correction (rad)
static uint8_t sensor_mag_axes_mode = 0;        // current axes-alignment selector

void connection_update_sensor_mag_diag(const float m_device[3],
                                       const float m_earth[3],
                                       float delta,
                                       uint8_t mag_axes_mode)
{
	memcpy(sensor_mag_device, m_device, sizeof(sensor_mag_device));
	memcpy(sensor_mag_earth,  m_earth,  sizeof(sensor_mag_earth));
	sensor_vqf_delta = delta;
	sensor_mag_axes_mode = mag_axes_mode;
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
	if (temp < -38.5f)
		sensor_temp = 1;
	else if (temp > 88.5f)
		sensor_temp = 255;
	else
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
}

static int64_t timeout_time = INT64_MAX;

void connection_update_sensor_timeout_time(int64_t timeout)
{
	timeout_time = timeout;
}

void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV) // format for packet send
{
	if (!battery_available) // No battery, and voltage is <=1500mV
	{
		batt = 0;
		batt_v = 0;
		return;
	}

	battery_pptt /= 100;
	batt = battery_pptt;
	batt |= 0x80; // battery_available, server will show a battery indicator

	if (charged) // 255, server will show fully charged indicator (not yet)
		batt = 255;

	if (plugged) // Charging
		battery_mV = MAX(battery_mV, 4310); // server will show a charging indicator

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) // Very dead but it is what it is
		batt_v = 0;
	else if (battery_mV > 255)
		batt_v = 255;
	else
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
}

void connection_update_status(int status)
{
	tracker_status = status;
	tracker_svr_status = get_server_constant_tracker_status(status);
}

static int64_t button_update_time = 0;

void connection_update_button(int button)
{
	tracker_button = button;
	button_update_time = k_uptime_get();
}

static bool shutdown = false;

void connection_set_shutdown(void)
{
	shutdown = true;
}

//|type    |priority|motion  |precise |interval|description
//|TX     0|       4|        |        |     100|device info ("info")
//|TX     1|       3|*       |*       |       -|full precision quat and accel
//|TX     2|       1|*       |        |     100|reduced precision quat and accel with battery, temp, and rssi ("info")
//|TX     3|       6|        |        |    1000|status ("status")
//|TX     4|       0|*       |*       |     200|full precision quat and magnetometer
//|TX     5|       7|        |        |    1000|runtime ("status2")
//|TX     6|       5|*       |        |     100|reduced precision quat and accel with button and sleep time ("info2")
//|TX     7|       2|        |        |     100|button and sleep time ("info2")

// precise: priority override; interval: target interval in milliseconds

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|TX     0|id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv----|imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|TX     1|id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|TX     2|id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|TX     3|id      |svr_stat|status  |resv----------------------------------------------------------------------------------------------|rssi    |
//|TX     4|id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|TX     5|id      |runtime                                                                |resv----------------------------------------|rssi    |
//|TX     6|id      |button  |sleeptime        |resv-------------------------------------------------------------------------------------|rssi    |
//|TX     7|id      |button  |sleeptime        |q_buf                              |a0               |a1               |a2               |rssi    |

// runtime is in microseconds (overkill), sleeptime is in milliseconds (overkill but less)

void connection_write_packet_0() // device info
{
	uint8_t data[16] = {0};
	data[0] = 0; // packet 0
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	data[5] = FW_BOARD; // brd_id
	data[6] = FW_MCU; // mcu_id
	data[7] = 0; // resv
	data[8] = imu_id; // imu_id
	data[9] = mag_id; // mag_id
	uint16_t *buf = (uint16_t *)&data[10];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[12] = FW_VERSION_MAJOR & 255; // fw_major
	data[13] = FW_VERSION_MINOR & 255; // fw_minor
	data[14] = FW_VERSION_PATCH & 255; // fw_patch
	data[15] = 0; // rssi (supplied by receiver)
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_1() // full precision quat and accel
{
	uint8_t data[16] = {0};
	data[0] = 1; // packet 1
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]); // ±1.0
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	buf[4] = TO_FIXED_7(sensor_a[0]); // range is ±256m/s² or ±26.1g
	buf[5] = TO_FIXED_7(sensor_a[1]);
	buf[6] = TO_FIXED_7(sensor_a[2]);
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_2() // reduced precision quat and accel with battery, temp, and rssi
{
	uint8_t data[16] = {0};
	data[0] = 2; // packet 2
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);

//	v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
//	v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
//	v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);
//	for (int i = 0; i < 3; i++)
//	v[i] = v[i] * 2 - 1;
//	float q[4] = {0};
//	q_iem(v, q); // inverse exponential map

	uint16_t *buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0; // rssi (supplied by receiver)
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	data[15] = 0; // rssi (supplied by receiver)
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_4() // RFT: full precision quat + d (real-time)
{
	uint8_t data[16] = {0};
	data[0] = 4; // packet 4
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]);
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	// 6 trailing bytes: VQF disturbance angle d (Q11, ±16 rad), 4 reserved.
	// raw_mag + bias moved to packet 8 (low-rate, on-demand).
	buf[4] = TO_FIXED_11(sensor_diag_dis_angle);
	buf[5] = 0;
	buf[6] = 0;
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

// Packet 8: raw mag + tracker bias + cal_done flag. Sent on demand only when
// PC requests via STREAM_RAW_MAG=on (e.g., user pressed Reset to recalibrate).
// Layout: [0]=8, [1]=id, [2..7]=raw mag (3×Q11), [8..13]=bias (3×Q11),
// [14]=cal_done (0/1), [15]=reserved.
void connection_write_packet_9()
{
	// Layout: [2..7]=mag_device (Q11), [8..13]=magEarth (Q11),
	//         [14]=delta as int8 Q5 (range ±3.97rad, resolution 1.8°),
	//         [15]=mag_axes_mode (uint8, 0..7).
	uint8_t data[16] = {0};
	data[0] = 9;
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_11(sensor_mag_device[0]);
	buf[1] = TO_FIXED_11(sensor_mag_device[1]);
	buf[2] = TO_FIXED_11(sensor_mag_device[2]);
	buf[3] = TO_FIXED_11(sensor_mag_earth[0]);
	buf[4] = TO_FIXED_11(sensor_mag_earth[1]);
	buf[5] = TO_FIXED_11(sensor_mag_earth[2]);
	// delta as int8 Q5: clamp & scale by 32. Range fits ±π fine.
	float d = sensor_vqf_delta * 32.0f;
	if (d >  127.0f) d =  127.0f;
	if (d < -128.0f) d = -128.0f;
	data[14] = (uint8_t)(int8_t)d;
	data[15] = sensor_mag_axes_mode;
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get();
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data);
}

void connection_write_packet_8()
{
	uint8_t data[16] = {0};
	data[0] = 8;
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_11(sensor_raw_mag[0]);
	buf[1] = TO_FIXED_11(sensor_raw_mag[1]);
	buf[2] = TO_FIXED_11(sensor_raw_mag[2]);
	buf[3] = TO_FIXED_11(sensor_mag_bias[0]);
	buf[4] = TO_FIXED_11(sensor_mag_bias[1]);
	buf[5] = TO_FIXED_11(sensor_mag_bias[2]);
	data[14] = sensor_mag_cal_done;
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get();
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data);
}

void connection_write_packet_5() // runtime
{
	uint8_t data[16] = {0};
	data[0] = 5; // packet 5
	data[1] = tracker_id;
	int64_t *buf = (int64_t *)&data[2];
	if (sys_get_valid_battery_pptt() >= 0)
		*buf = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	else
		*buf = -1; // no valid reading yet, but previous estimate may still be valid
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_6() // reduced precision quat and accel with button and sleep time
{
	uint8_t data[16] = {0};
	data[0] = 6; // packet 6
	data[1] = tracker_id;
	data[2] = tracker_button;
	uint16_t *buf = (uint16_t *)&data[3];
	if (shutdown)
		*buf = 1;
	else
		*buf = timeout_time < 1 ? 1 : timeout_time;
	if (k_ticks_to_ms_floor64(sys_get_battery_remaining_time_estimate()) < 60000 && timeout_time == UINT16_MAX)
		timeout_time = UINT16_MAX - 1;
	data[15] = 0; // rssi (supplied by receiver)
	if (tracker_button && k_uptime_get() > button_update_time + 1000) // attempt to send button press for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_7() // button and sleep time
{
	uint8_t data[16] = {0};
	data[0] = 7; // packet 7
	data[1] = tracker_id;
	data[2] = tracker_button;
	uint16_t *buf = (uint16_t *)&data[3];
	if (shutdown)
		*buf = 1;
	else
		*buf = timeout_time < 1 ? 1 : timeout_time;
	if (k_ticks_to_ms_floor64(sys_get_battery_remaining_time_estimate()) < 60000 && timeout_time == UINT16_MAX)
		timeout_time = UINT16_MAX - 1;
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);
	buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0; // rssi (supplied by receiver)
	if (tracker_button && k_uptime_get() > button_update_time + 1000) // attempt to send button press for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
	int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(100));
	if (ret) {
		LOG_ERR("Failed mutex lock");
		return;
	}
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

// TODO: get radio channel from receiver
// TODO: new packet format

// TODO: use timing from IMU to get actual delay in tracking
// TODO: aware of sensor state? error status, timing/phase, maybe "send_precise_quat"

// TODO: queuing, status is lowest priority, info low priority, existing data highest priority (from sensor loop)

// TODO: queue packets directly for HID, or maintain separate loop while connected by USB

static int64_t last_info_time = 0;
static int64_t last_info2_time = 0;
static int64_t last_status_time = 0;
static int64_t last_status2_time = 0;

// RFT TDMA: connection_thread is now slot-driven. Each iteration blocks on
// tdma_anchor_wait_slot() which fires:
//   - on the per-slot hardware timer when we have a TIMING ACK anchor
//     (TRACKING state, slot-locked to the receiver),
//   - on the 1 s ALOHA timer when the slot is set but no TIMING ACK has
//     arrived recently (DISCOVERY state — slow broadcast, lets the
//     receiver lock us in without contributing meaningfully to collisions),
//   - or just returns from a long sleep when the slot is unset (SILENT —
//     no TX at all; the receiver decides our slot via PKT_SET_SLOT_INDEX
//     after pair).
//
// Per slot we always TX pkt1 (the receiver's primary signal). Once every
// TDMA_HOUSEKEEPING_PERIOD_MAG_D cycles a secondary packet rides along:
// pkt4 (mag diagnostic) by default; pkt8 / pkt9 (raw mag stream) chained
// into the next two mag ticks when the PC has requested the mag stream.
// Battery / device-info goes out every TDMA_HOUSEKEEPING_PERIOD_BATTERY
// cycles (~10 s @ 30 Hz). The receiver-side housekeeping rotation key is
// `cycle_counter % period == my_slot_index` so the 10 trackers stagger.
void connection_thread(void)
{
	uint8_t data_copy[21];
	enum { STREAM_NONE = 0, STREAM_NEXT_PKT8, STREAM_NEXT_PKT9 } stream_state = STREAM_NONE;

	while (1)
	{
		uint32_t cycle = tdma_anchor_wait_slot();
		tdma_anchor_state_t st = tdma_anchor_get_state();
		if (st == TDMA_ANCHOR_SILENT) continue;  /* unset slot — no TX */

		uint8_t my_idx = tdma_anchor_get_slot();

		/* === Primary: pkt1 (full-precision quat + accel). ===
		 * Always sent if fresh fusion data is available. The receiver
		 * gates everything on these arriving — losing one is the worst
		 * outcome of any per-slot trade-off, so it goes out first. */
		bool sent_primary = false;
		if (last_data_time != 0) {
			int ret = k_mutex_lock(&data_buffer_mutex, K_MSEC(10));
			if (!ret) {
				last_data_time = 0;
				memcpy(data_copy, data_buffer, sizeof(data_copy));
				k_mutex_unlock(&data_buffer_mutex);
				data_copy[20] = packet_sequence++;
				uint32_t *crc_ptr = (uint32_t *)&data_copy[16];
				*crc_ptr = crc32_k_4_2_update(0x93a409eb, data_copy, 16);
				esb_write(data_copy);
				sent_primary = true;
			}
		}

		/* In DISCOVERY mode the housekeeping rotation has no useful
		 * cycle anchor — we'd be picking a slot blindly. Just send the
		 * primary (or nothing if fusion hasn't produced data yet) and
		 * wait for the receiver to lock us in. */
		if (st != TDMA_ANCHOR_TRACKING) continue;

		/* === Secondary: at most one housekeeping packet per slot,
		 * rotated by cycle_counter so the 10 trackers stagger. The
		 * busy_wait lets the primary TX leave the ESB FIFO before the
		 * secondary's esb_write() flushes it — without this, calling
		 * esb_write() twice back-to-back drops the first. ESB ramp-up
		 * is ~140 µs and 16 B at 2 Mbps is ~80 µs, so 300 µs is a
		 * conservative margin. (A non-flushing esb_write_chain would
		 * be cleaner but needs care in the ESB driver layer.) */
		bool want_secondary = false;
		if (stream_state == STREAM_NEXT_PKT8 ||
		    stream_state == STREAM_NEXT_PKT9) {
			want_secondary = true;
		} else if ((cycle % TDMA_HOUSEKEEPING_PERIOD_BATTERY) == my_idx) {
			want_secondary = true;
		} else if ((cycle % TDMA_HOUSEKEEPING_PERIOD_MAG_D) == my_idx) {
			want_secondary = true;
		}
		if (!want_secondary) continue;

		if (sent_primary) k_busy_wait(300);   /* let pkt1 clear FIFO */

		extern volatile bool rft_mag_stream_enabled;
		if (stream_state == STREAM_NEXT_PKT8) {
			connection_write_packet_8();
			stream_state = rft_mag_stream_enabled ? STREAM_NEXT_PKT9 : STREAM_NONE;
		} else if (stream_state == STREAM_NEXT_PKT9) {
			connection_write_packet_9();
			stream_state = STREAM_NONE;
		} else if ((cycle % TDMA_HOUSEKEEPING_PERIOD_BATTERY) == my_idx) {
			/* device info incl. battery + temp + RSSI */
			connection_write_packet_2();
		} else if ((cycle % TDMA_HOUSEKEEPING_PERIOD_MAG_D) == my_idx) {
			connection_write_packet_4();
			/* If the PC has the mag stream open, chain pkt8/pkt9 into
			 * the next two mag ticks. They share data_buffer with pkt4
			 * so we can't fire them in the same slot. */
			if (rft_mag_stream_enabled) stream_state = STREAM_NEXT_PKT8;
		}
	}
}
