/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

float utils_map_angle(float angle, float min, float max);
void utils_deadband(float *value, float tres, float max);
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);
float utils_interpolate_angles_rad(float a1, float a2, float weight_a1);
float utils_middle_of_3(float a, float b, float c);
int utils_middle_of_3_int(int a, int b, int c);
float utils_fast_atan2(float y, float x);
float utils_fast_sin(float angle);
float utils_fast_cos(float angle);
void utils_fast_sincos(float angle, float *sin, float *cos);
void utils_fast_sincos_better(float angle, float *sin, float *cos);
float utils_min_abs(float va, float vb);
float utils_max_abs(float va, float vb);
void utils_byte_to_binary(int x, char *b);
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode);
uint32_t utils_crc32c(uint8_t *data, uint32_t len);
void utils_fft32_bin0(float *real_in, float *real, float *imag);
void utils_fft32_bin1(float *real_in, float *real, float *imag);
void utils_fft32_bin2(float *real_in, float *real, float *imag);
void utils_fft16_bin0(float *real_in, float *real, float *imag);
void utils_fft16_bin1(float *real_in, float *real, float *imag);
void utils_fft16_bin2(float *real_in, float *real, float *imag);
void utils_fft8_bin0(float *real_in, float *real, float *imag);
void utils_fft8_bin1(float *real_in, float *real, float *imag);
void utils_fft8_bin2(float *real_in, float *real, float *imag);
float utils_batt_liion_norm_v_to_capacity(float norm_v);
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
		unsigned int *buffer_index, unsigned int filter_len, uint16_t sample);
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse);

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)				(((x) < 0.0F) ? -1.0F : 1.0F)

// Squared
#define SQ(x)				((x) * (x))

// Two-norm of 2D vector
//#define NORM2(x,y)		(sqrt(SQ(x) + SQ(y)))
#define NORM2_f(x,y)		(sqrtf(SQ(x) + SQ(y)))

// nan and infinity check for floats
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0F : x)

// Handy conversions for radians/degrees and RPM/radians-per-second
#define UTILS_PI_F (3.14159265358979323846F)
#define DEG2RAD_f(deg) ((deg) * (UTILS_PI_F / 180.0F))
#define RAD2DEG_f(rad) ((rad) * (180.0F / UTILS_PI_F))
#define RPM2RADPS_f(rpm) ((rpm) * ((2.0F * UTILS_PI_F) / 60.0F))
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (60.0F / (2.0F * UTILS_PI_F)))

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

// For double precision literals
#define D(x) 				((double)x##L)

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

/**
 * A fast approximation of a moving average filter with N samples. See
 * https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
 * https://en.wikipedia.org/wiki/Exponential_smoothing
 *
 * It is not entirely the same as it behaves like an IIR filter rather than a FIR filter, but takes
 * much less memory and is much faster to run.
 */
#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)	UTILS_LP_FAST(value, sample, 2.0 / ((N) + 1.0))

// Constants
#define ONE_BY_SQRT3			(0.57735026919)
#define TWO_BY_SQRT3			(2.0f * 0.57735026919)
#define SQRT3_BY_2				(0.86602540378)
#define COS_30_DEG				(0.86602540378)
#define SIN_30_DEG				(0.5)
#define COS_MINUS_30_DEG		(0.86602540378)
#define SIN_MINUS_30_DEG		(-0.5)
#define ONE_BY_SQRT2			(0.7071067811865475)

// Tables
extern const float utils_tab_sin_32_1[];
extern const float utils_tab_sin_32_2[];
extern const float utils_tab_cos_32_1[];
extern const float utils_tab_cos_32_2[];

// Inline functions
static inline void utils_step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
static inline void utils_norm_angle(float *angle) {
//	*angle = fmodf(*angle, 360.0);
//	if (*angle < 0.0) {
//		*angle += 360.0;
//	}

	// This is much faster than fmodf
	while (*angle < 0.0F) { *angle += 360.0F; }
	while (*angle >= 360.0F) { *angle -= 360.0F; }
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
static inline void utils_norm_angle_rad(float *angle) {
	while (*angle < -UTILS_PI_F) { *angle += 2.0F * UTILS_PI_F; }
	while (*angle >=  UTILS_PI_F) { *angle -= 2.0F * UTILS_PI_F; }
}

static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_truncate_number_int(int *number, int min, int max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_truncate_number_uint32(uint32_t *number, uint32_t min, uint32_t max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_truncate_number_abs(float *number, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < -max) {
		*number = -max;
	}
}

static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Truncate the magnitude of a vector.
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
static inline bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = NORM2_f(*x, *y);
	max = fabsf(max);

	if (mag < 1e-10F) {
		mag = 1e-10F;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
static inline float utils_angle_difference(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -180.0F) difference += 2.0F * 180.0F;
	while (difference > 180.0F) difference -= 2.0F * 180.0F;
	return difference;
}

/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
static inline float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -UTILS_PI_F) difference += 2.0F * UTILS_PI_F;
	while (difference > UTILS_PI_F) difference -= 2.0F * UTILS_PI_F;
	return difference;
}

#endif  /* UTILS_MATH_H_ */
