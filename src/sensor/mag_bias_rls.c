/*
	RFT mag bias estimator with fixed sphere radius — see mag_bias_rls.h
*/
#include "mag_bias_rls.h"

#include <math.h>
#include <string.h>

void rls_sphere_init(rls_sphere_t *rls, float radius_g, float lr)
{
	memset(rls->c, 0, sizeof(rls->c));
	rls->radius = radius_g;
	rls->lr = lr;
	rls->samples = 0;
}

void rls_sphere_update(rls_sphere_t *rls, const float m[3])
{
	// Reject garbage input
	for (int i = 0; i < 3; i++) {
		if (!isfinite(m[i]) || fabsf(m[i]) > 10.0f) return;
	}

	float dx = m[0] - rls->c[0];
	float dy = m[1] - rls->c[1];
	float dz = m[2] - rls->c[2];
	float dist = sqrtf(dx*dx + dy*dy + dz*dz);
	if (dist < 0.001f) return; // avoid divide by zero, no info anyway

	// Gradient step: pull center toward (or away from) the sample so that
	// the new distance moves toward radius. factor = 2*lr*(1 - R/dist).
	// dist > R → factor > 0 → c moves toward m (sample is "outside")
	// dist < R → factor < 0 → c moves away from m (sample is "inside")
	float factor = 2.0f * rls->lr * (1.0f - rls->radius / dist);
	rls->c[0] += factor * dx;
	rls->c[1] += factor * dy;
	rls->c[2] += factor * dz;

	// Defensive clamp on bias (hard iron physically < 5G)
	for (int i = 0; i < 3; i++) {
		if (!isfinite(rls->c[i]) || fabsf(rls->c[i]) > 5.0f) {
			rls->c[0] = rls->c[1] = rls->c[2] = 0.0f;
			break;
		}
	}

	rls->samples++;
}

void rls_sphere_get_bias(const rls_sphere_t *rls, float bias[3])
{
	// Warmup: don't apply bias until we have enough samples
	if (rls->samples < 50) {
		bias[0] = bias[1] = bias[2] = 0.0f;
		return;
	}
	bias[0] = rls->c[0];
	bias[1] = rls->c[1];
	bias[2] = rls->c[2];
}

float rls_sphere_get_radius(const rls_sphere_t *rls)
{
	return rls->radius;
}
