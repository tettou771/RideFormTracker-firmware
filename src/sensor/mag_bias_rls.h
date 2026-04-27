/*
	RFT online hard iron estimator for magnetometer — see MAG_CALIBRATION.md

	Recursive Least Squares sphere fit: incrementally estimates the center
	(hard iron bias) and squared radius of a sphere from magnetometer samples.

	Model: |m - c|^2 = r^2
	   →  m.x^2 + m.y^2 + m.z^2 = 2 c.x m.x + 2 c.y m.y + 2 c.z m.z + (r^2 - |c|^2)
	   →  y = phi^T theta   with   phi = [2mx, 2my, 2mz, 1]ᵀ
	                                theta = [cx, cy, cz, r^2 - |c|^2]ᵀ

	O(1) per sample, ~100 bytes state, ~1.5us on M4F @ 64MHz.
*/
#ifndef SLIMENRF_MAG_BIAS_RLS_H
#define SLIMENRF_MAG_BIAS_RLS_H

typedef struct {
	float theta[4];   // [cx, cy, cz, r^2 - |c|^2]
	float P[4][4];    // covariance matrix
	float lambda;     // forgetting factor (0.9999 ≈ 10k sample memory)
	unsigned long samples;
} rls_sphere_t;

void rls_sphere_init(rls_sphere_t *rls, float lambda, float p0);
void rls_sphere_update(rls_sphere_t *rls, const float m[3]);
void rls_sphere_get_bias(const rls_sphere_t *rls, float bias[3]);
float rls_sphere_get_radius(const rls_sphere_t *rls);

#endif
