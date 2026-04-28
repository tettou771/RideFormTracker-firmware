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
#ifndef SLIMENRF_CONNECTION
#define SLIMENRF_CONNECTION

void connection_clocks_request_start(void);
void connection_clocks_request_start_delay_us(uint32_t delay_us);
void connection_clocks_request_stop(void);
void connection_clocks_request_stop_delay_us(uint32_t delay_us);

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);

void connection_update_sensor_ids(int imu_id, int mag_id);
void connection_update_sensor_data(float *q, float *a, int64_t data_time); // ticks
void connection_update_sensor_mag(float *m);

// RFT: raw (uncorrected) mag for PC-side calibration. Captured here, sent in
// packet 8 when the PC has requested streaming.
void connection_update_sensor_raw_mag(const float m[3]);

// RFT: tracker-side bias + cal-done flag, sent alongside raw_mag in packet 8.
void connection_update_sensor_mag_bias(const float bias[3], bool cal_done);

// VQF heading disturbance angle (rad, ±π). Sent in packet 4 every frame.
// (biasMag and sphereR args are ignored — they used to share this slot but
// are now PC-side computations.)
void connection_update_sensor_diag(float disAngle, float biasMag, float sphereR);
void connection_update_sensor_temp(float temp);
void connection_update_sensor_timeout_time(int64_t timeout);
void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV);
void connection_update_status(int status);
void connection_update_button(int button);

void connection_set_shutdown(void);

void connection_write_packet_0(void);
void connection_write_packet_1(void);
void connection_write_packet_2(void);
void connection_write_packet_3(void);
void connection_write_packet_4(void);
void connection_write_packet_5(void);
void connection_write_packet_8(void);  // raw mag + bias (on-demand stream)

#endif
