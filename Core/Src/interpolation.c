/*
 * Different interpolation methods, taken from
 * https://paulbourke.net/miscellaneous/interpolation/
 */
#include "interpolation.h"

static inline float32_t lin_interpolate(
	float32_t y1, float32_t y2, float32_t mu)
{
	return y1 * (1 - mu) + y2 * mu;
}

static inline float32_t cos_interpolate(
	float32_t y1, float32_t y2, float32_t mu)
{
	float32_t mu2;

	mu2 = (1 - arm_cos_f32(mu * PI)) / 2;
	return y1 * (1 - mu2) + y2 * mu2;
}

static inline float32_t cubic_interpolate(
	float32_t y0, float32_t y1,
	float32_t y2, float32_t y3,
	float32_t mu)
{
	float32_t a0, a1, a2, a3, mu2;

	mu2 = mu * mu;
	a0 = y3 - y2 - y0 + y1;
	a1 = y0 - y1 - a0;
	a2 = y2 - y0;
	a3 = y1;

	return a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3;
}

bool interpolate(const float32_t *in, size_t in_sz,
				 float32_t *out, size_t out_sz,
				 enum interpolation_method method)
{
	size_t n_seg, n_points_per_seg, n_points_extra, n_points;
	size_t i_seg, i_p, i_out;

	if (in == NULL || out == NULL)
		return false;

	if (in_sz < 2 || out_sz <= in_sz)
		return false;

	/* Except the last one */
	n_points = out_sz - 1;
	n_seg = in_sz - 1;
	n_points_per_seg = n_points / n_seg;
	n_points_extra = n_points - n_points_per_seg * n_seg;

	for (i_seg = 0, i_out = 0; i_seg < n_seg; i_seg++) {
		float32_t p_beg, p_end, p0, p1;
		size_t n;

		n = n_points_per_seg;
		if (n_points_extra > 0) {
			n_points_extra -= 1;
			n += 1;
		}

		p0 = in[i_seg];
		p1 = in[i_seg + 1];
		p_beg = (i_seg == 0 ? p0 : in[i_seg - 1]);
		p_end = (i_seg + 1 == n_seg ? p1 : in[i_seg + 1]);

		out[i_out++] = p0;

		for (i_p = 0; i_p < n - 1; i_p++) {
			float32_t mu, p;

			mu = 1.0 / n * (i_p + 1);
			switch(method) {
			case lin_interpolation:
				p = lin_interpolate(p0, p1, mu);
				break;
			case cos_interpolation:
				p = cos_interpolate(p0, p1, mu);
				break;
			case cubic_interpolation:
				p = cubic_interpolate(p_beg, p0, p1, p_end, mu);
				break;
			default:
				return false;
			}

			out[i_out++] = p;
		}

		if (i_seg + 1 == n_seg)
			/* Don't forget the last one point */
			out[i_out++] = p1;
	}

	return true;
}
