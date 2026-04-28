/*
	RFT online hard iron estimator with voxel-binned, fixed-radius fitting.

	Why fixed radius: the geomagnetic field magnitude is roughly constant
	within a region (~0.45G in Tokyo). Locking R reduces the fit from 4
	unknowns to 3 (just the sphere center) and makes "false convergence
	on a small sphere centered on a static pose" impossible.

	Why voxel binning: a naive gradient descent over-weights regions
	where the user shakes a lot and under-weights the rest, producing
	biased fits when the motion is uneven. Each unique voxel of raw mag
	space contributes equally regardless of how many samples landed there.

	Algorithm per sample:
	  1. Quantize sample to voxel coordinate (block size ~0.3 G).
	  2. If voxel exists in store, replace its sample with the new one
	     (newest wins). Else add to store; if full, round-robin evict.
	  3. Apply one gradient step using a stored voxel's sample
	     (round-robin across voxels). Each voxel contributes equally
	     over time.

	  Gradient step:  c += 2 * lr * (1 - R / |m - c|) * (m - c)
*/
#ifndef SLIMENRF_MAG_BIAS_RLS_H
#define SLIMENRF_MAG_BIAS_RLS_H

#include <stdint.h>

#define MAG_RLS_MAX_VOXELS 32
#define MAG_RLS_VOXEL_SIZE 0.3f /* Gauss */

typedef struct {
	int16_t key;       // packed (kx, ky, kz)
	float sample[3];
} mag_voxel_t;

typedef struct {
	float c[3];                                  // estimated bias (sphere center)
	float radius;                                // fixed sphere radius
	float lr;                                    // gradient learning rate
	mag_voxel_t voxels[MAG_RLS_MAX_VOXELS];
	int n_voxels;
	int next_evict;                              // round-robin eviction index
	int gradient_idx;                            // round-robin gradient sample index
	unsigned long samples;
	// Octant coverage: bit i = 1 if a sample with sign pattern i has been seen.
	// Index = (mx>0)<<0 | (my>0)<<1 | (mz>0)<<2. Used to require samples spread
	// across the sphere — a sphere fit with samples concentrated in one
	// hemisphere can converge to a wrong center even at fixed R.
	uint8_t octant_mask;
} rls_sphere_t;

void rls_sphere_init(rls_sphere_t *rls, float radius_g, float lr);
void rls_sphere_update(rls_sphere_t *rls, const float m[3]);
void rls_sphere_get_bias(const rls_sphere_t *rls, float bias[3]);
void rls_sphere_set_bias(rls_sphere_t *rls, const float bias[3]); // for NVS load
float rls_sphere_get_radius(const rls_sphere_t *rls);
int rls_sphere_get_voxel_count(const rls_sphere_t *rls);
int rls_sphere_get_octant_count(const rls_sphere_t *rls);

#endif
