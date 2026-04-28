/*
	RFT online hard iron estimator for magnetometer.

	Fixed-radius sphere fitting. The geomagnetic field magnitude is known
	(roughly constant in a given region — ≈0.45G in Tokyo), so we only
	estimate the sphere center (the hard iron offset). Three unknowns
	instead of four — converges with much less data and refuses to settle
	on a tiny "false" sphere centered on whatever pose the user happened
	to leave the tracker in.

	Algorithm: per sample, gradient descent on the squared radius error:
	  e = |m - c| - R_target
	  c += 2 * lr * (1 - R_target/|m - c|) * (m - c)

	~10 bytes state, microseconds per sample.
*/
#ifndef SLIMENRF_MAG_BIAS_RLS_H
#define SLIMENRF_MAG_BIAS_RLS_H

typedef struct {
	float c[3];           // estimated bias (sphere center)
	float radius;         // fixed sphere radius (geomag magnitude)
	float lr;             // gradient descent learning rate
	unsigned long samples;
} rls_sphere_t;

// radius_g: expected geomagnetic field magnitude in Gauss (≈0.45 in Tokyo).
// lr:       learning rate (typical 0.01-0.05). Higher = faster convergence,
//           more noise sensitivity.
void rls_sphere_init(rls_sphere_t *rls, float radius_g, float lr);
void rls_sphere_update(rls_sphere_t *rls, const float m[3]);
void rls_sphere_get_bias(const rls_sphere_t *rls, float bias[3]);
float rls_sphere_get_radius(const rls_sphere_t *rls);

#endif
