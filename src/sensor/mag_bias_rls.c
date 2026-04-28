/*
	RFT mag bias estimator: voxel-binned, fixed-radius sphere fit.
	See mag_bias_rls.h for the rationale.
*/
#include "mag_bias_rls.h"

#include <math.h>
#include <string.h>

void rls_sphere_init(rls_sphere_t *rls, float radius_g, float lr)
{
	memset(rls, 0, sizeof(*rls));
	rls->radius = radius_g;
	rls->lr = lr;
}

static int16_t voxel_key(const float m[3])
{
	int8_t kx = (int8_t)floorf(m[0] / MAG_RLS_VOXEL_SIZE);
	int8_t ky = (int8_t)floorf(m[1] / MAG_RLS_VOXEL_SIZE);
	int8_t kz = (int8_t)floorf(m[2] / MAG_RLS_VOXEL_SIZE);
	// 5-bit signed coordinate per axis (-16..15) is plenty for raw mag space.
	return (int16_t)((kx & 0x1F) | ((ky & 0x1F) << 5) | ((kz & 0x1F) << 10));
}

static int find_voxel(const rls_sphere_t *rls, int16_t key)
{
	for (int i = 0; i < rls->n_voxels; i++) {
		if (rls->voxels[i].key == key) return i;
	}
	return -1;
}

static void apply_gradient(rls_sphere_t *rls, const float sample[3])
{
	float dx = sample[0] - rls->c[0];
	float dy = sample[1] - rls->c[1];
	float dz = sample[2] - rls->c[2];
	float dist = sqrtf(dx*dx + dy*dy + dz*dz);
	if (dist < 0.001f) return;
	float factor = 2.0f * rls->lr * (1.0f - rls->radius / dist);
	rls->c[0] += factor * dx;
	rls->c[1] += factor * dy;
	rls->c[2] += factor * dz;
}

void rls_sphere_update(rls_sphere_t *rls, const float m[3])
{
	// Reject garbage input
	for (int i = 0; i < 3; i++) {
		if (!isfinite(m[i]) || fabsf(m[i]) > 10.0f) return;
	}

	// Mark which octant relative to the current bias estimate this sample
	// fell in. Using (m - c) instead of raw m is critical when hard iron is
	// large: e.g. a +0.5G x-bias would lock raw m_x to always-positive and
	// pin coverage to 4 octants forever. Centered samples expand into all
	// 8 octants once c converges toward the true center.
	float dx = m[0] - rls->c[0];
	float dy = m[1] - rls->c[1];
	float dz = m[2] - rls->c[2];
	uint8_t oct = (dx > 0 ? 1 : 0) | (dy > 0 ? 2 : 0) | (dz > 0 ? 4 : 0);
	rls->octant_mask |= (uint8_t)(1u << oct);

	int16_t key = voxel_key(m);
	int idx = find_voxel(rls, key);
	if (idx >= 0) {
		// Existing voxel — newest wins
		memcpy(rls->voxels[idx].sample, m, sizeof(float) * 3);
	} else if (rls->n_voxels < MAG_RLS_MAX_VOXELS) {
		rls->voxels[rls->n_voxels].key = key;
		memcpy(rls->voxels[rls->n_voxels].sample, m, sizeof(float) * 3);
		rls->n_voxels++;
	} else {
		// Full: round-robin evict (replace oldest stored entry)
		rls->voxels[rls->next_evict].key = key;
		memcpy(rls->voxels[rls->next_evict].sample, m, sizeof(float) * 3);
		rls->next_evict = (rls->next_evict + 1) % MAG_RLS_MAX_VOXELS;
	}

	// Apply ONE gradient step using a stored voxel's sample, round-robin.
	// This way each voxel contributes equally over time regardless of how
	// many raw samples landed in it (the user shaking lots in one direction
	// doesn't dominate the fit).
	if (rls->n_voxels > 0) {
		rls->gradient_idx = (rls->gradient_idx + 1) % rls->n_voxels;
		apply_gradient(rls, rls->voxels[rls->gradient_idx].sample);
	}

	// Defensive clamp on bias
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
	if (rls->samples < 50) {
		bias[0] = bias[1] = bias[2] = 0.0f;
		return;
	}
	bias[0] = rls->c[0];
	bias[1] = rls->c[1];
	bias[2] = rls->c[2];
}

void rls_sphere_set_bias(rls_sphere_t *rls, const float bias[3])
{
	memcpy(rls->c, bias, sizeof(float) * 3);
	// Mark as warmed up so get_bias returns the value immediately
	if (rls->samples < 50) rls->samples = 50;
	// Mark as fully covered too — NVS bias was earned by passing the coverage
	// gate at save time, no need to make the user reshake to re-prove it.
	rls->octant_mask = 0xFF;
}

float rls_sphere_get_radius(const rls_sphere_t *rls)
{
	return rls->radius;
}

int rls_sphere_get_voxel_count(const rls_sphere_t *rls)
{
	return rls->n_voxels;
}

int rls_sphere_get_octant_count(const rls_sphere_t *rls)
{
	return __builtin_popcount(rls->octant_mask);
}
