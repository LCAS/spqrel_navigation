#pragma once
#include "path_map.h"

namespace srrg_core{
  
  void grayMap2indices(IntImage& dest,
		       const UnsignedCharImage& src,
		       unsigned char occ_threshold,
		       unsigned char free_threshold);

  void indices2distances(FloatImage& distances,
			 const IntImage& indices,
			 float resolution,
			 float max_distance);

  void distances2cost(FloatImage& dest,
		      const FloatImage& src,
		      float robot_radius,
		      float safety_region,
		      float min_cost,
		      float max_cost);

}
