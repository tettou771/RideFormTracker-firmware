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
#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>

#include "retained.h"
#include "config.h"
#include "thread_priority.h"

/* Sensor gyroscope, accelerometer, and magnetometer axes should align to the IMU body axes
 * SENSOR_QUATERNION_CORRECTION should align the sensor to the device following Android convention
 * On flat surface / face up:
 * Left from the perspective of the device / right from your perspective is +X
 * Front side (facing up) is +Z
 * Mounted on body / standing up:
 * Top side of the device is +Y
 * Front side (facing out) is +Z
 */

// TODO: not matching anymore
#if defined(CONFIG_BOARD_SLIMEVRMINI_P1_UF2) || defined(CONFIG_BOARD_SLIMEVRMINI_P2_UF2)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -mx, mz, -my
#define SENSOR_QUATERNION_CORRECTION 0.7071f, 0.7071f, 0.0f, 0.0f
#endif

#if defined(CONFIG_BOARD_SLIMENRF_R1) || defined(CONFIG_BOARD_SLIMENRF_R2) || defined(CONFIG_BOARD_SLIMENRF_R3)
#define SENSOR_QUATERNION_CORRECTION 0.0f, 0.7071f, 0.7071f, 0.0f
#endif

// RFT v1.1: IIS2MDC is mounted rotated relative to LSM6DSV (pin1 markers
// differ). The macro maps mag-chip-local (mx,my,mz) into IMU-body axes.
// Try one of these (uncomment one); the right one is whichever makes
// the disagreement angle (`d` in Deck) stay near 0 while you rotate the
// tracker to different poses:
//   #define SENSOR_MAGNETOMETER_AXES_ALIGNMENT  mx,  my,  mz   // 0°
//   #define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -my,  mx,  mz   // 90° CCW around Z
//   #define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -mx, -my,  mz   // 180°
//   #define SENSOR_MAGNETOMETER_AXES_ALIGNMENT  my, -mx,  mz   // 270° CCW (= 90° CW)
// If d still drifts during pure tilt (no yaw), one axis is upside down:
//   try negating mz, or swapping the X/Y as above with -mz instead.
#if defined(CONFIG_BOARD_RFT_PCB_V1_1)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -my, -mx, mz  // datasheet-derived (default)
#endif

#ifndef SENSOR_MAGNETOMETER_AXES_ALIGNMENT
// mag axes alignment to sensor body
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, -mx, -mz
#endif
#ifndef SENSOR_QUATERNION_CORRECTION
// correction quat for sensor to mounting orientation
#define SENSOR_QUATERNION_CORRECTION 1.0f, 0.0f, 0.0f, 0.0f
#endif

#endif