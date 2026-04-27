/*
	RFT online hard iron estimator — see mag_bias_rls.h
*/
#include "mag_bias_rls.h"

#include <math.h>
#include <string.h>

void rls_sphere_init(rls_sphere_t *rls, float lambda, float p0)
{
	memset(rls->theta, 0, sizeof(rls->theta));
	memset(rls->P, 0, sizeof(rls->P));
	for (int i = 0; i < 4; i++)
		rls->P[i][i] = p0;
	rls->lambda = lambda;
	rls->samples = 0;
}

static int is_finite_vec(const float *v, int n)
{
	for (int i = 0; i < n; i++) {
		if (!isfinite(v[i]))
			return 0;
	}
	return 1;
}

void rls_sphere_update(rls_sphere_t *rls, const float m[3])
{
	// Reject garbage input (NaN/Inf/out-of-range)
	if (!is_finite_vec(m, 3))
		return;
	for (int i = 0; i < 3; i++) {
		if (fabsf(m[i]) > 10.0f) // sensor full scale is ±50G but hard iron shouldn't exceed ~5G
			return;
	}

	// Regressor phi = [2mx, 2my, 2mz, 1]
	const float phi[4] = { 2.0f * m[0], 2.0f * m[1], 2.0f * m[2], 1.0f };
	const float y = m[0]*m[0] + m[1]*m[1] + m[2]*m[2];

	// P_phi = P * phi  (4-vector)
	float P_phi[4];
	for (int i = 0; i < 4; i++) {
		float s = 0.0f;
		for (int j = 0; j < 4; j++)
			s += rls->P[i][j] * phi[j];
		P_phi[i] = s;
	}

	// denom = lambda + phi^T * P_phi
	float denom = rls->lambda;
	for (int i = 0; i < 4; i++)
		denom += phi[i] * P_phi[i];

	// Guard against division blow-up
	if (!isfinite(denom) || denom < 1e-6f)
		return;

	// Kalman-like gain K = P_phi / denom
	float K[4];
	for (int i = 0; i < 4; i++)
		K[i] = P_phi[i] / denom;

	// Error e = y - phi^T * theta
	float yhat = 0.0f;
	for (int i = 0; i < 4; i++)
		yhat += phi[i] * rls->theta[i];
	const float err = y - yhat;

	// Theta update: theta += K * e
	float new_theta[4];
	for (int i = 0; i < 4; i++)
		new_theta[i] = rls->theta[i] + K[i] * err;

	// Reject update if it produced non-finite results
	if (!is_finite_vec(new_theta, 4))
		return;

	for (int i = 0; i < 4; i++)
		rls->theta[i] = new_theta[i];

	// Covariance update: P = (P - K * P_phi^T) / lambda  (symmetric P assumption)
	const float inv_lambda = 1.0f / rls->lambda;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			rls->P[i][j] = (rls->P[i][j] - K[i] * P_phi[j]) * inv_lambda;
		}
	}

	// Periodic symmetrization to guard against numerical drift
	if ((++rls->samples % 1024) == 0) {
		for (int i = 0; i < 4; i++) {
			for (int j = i + 1; j < 4; j++) {
				float avg = 0.5f * (rls->P[i][j] + rls->P[j][i]);
				rls->P[i][j] = avg;
				rls->P[j][i] = avg;
			}
		}
	}
}

void rls_sphere_get_bias(const rls_sphere_t *rls, float bias[3])
{
	// Warmup: don't apply bias until we have enough samples
	if (rls->samples < 100) {
		bias[0] = 0.0f;
		bias[1] = 0.0f;
		bias[2] = 0.0f;
		return;
	}

	for (int i = 0; i < 3; i++) {
		float b = rls->theta[i];
		// Defensive: clamp to physical hard iron range, reject NaN/Inf
		if (!isfinite(b) || fabsf(b) > 3.0f)
			b = 0.0f;
		bias[i] = b;
	}
}

float rls_sphere_get_radius(const rls_sphere_t *rls)
{
	const float c2 = rls->theta[0]*rls->theta[0]
	               + rls->theta[1]*rls->theta[1]
	               + rls->theta[2]*rls->theta[2];
	const float r2 = rls->theta[3] + c2;
	return r2 > 0.0f ? sqrtf(r2) : 0.0f;
}
