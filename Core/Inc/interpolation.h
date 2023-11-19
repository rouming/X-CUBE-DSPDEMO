#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <stdbool.h>
#include "arm_math.h"


enum interpolation_method {
	lin_interpolation,
	cos_interpolation,
	cubic_interpolation,
};

bool interpolate(const float32_t *in, size_t in_sz,
				 float32_t *out, size_t out_sz,
				 enum interpolation_method method);

#endif /* INTERPOLATION_H */
