#include "path_map.h"
#include "dijkstra_path_search.h"
#include "distance_map_path_search.h"

namespace srrg_core {

  void grayMap2indices(IntImage& dest,
		       const UnsignedCharImage& src,
		       unsigned char occ_threshold,
		       unsigned char free_threshold) {
    int rows=src.rows;
    int cols=src.cols;
    dest.create(rows, cols);
    int num_occupied=0;
    int num_free=0;
    int num_unknown=0;
    for (int r=0; r<rows; ++r){
      int* dest_ptr=dest.ptr<int>(r);
      const unsigned char* src_ptr=src.ptr<const unsigned char>(r);
      for (int c=0; c<cols; ++c, ++dest_ptr, ++ src_ptr){
	unsigned char cell_value=*src_ptr;
	if (*src_ptr>free_threshold){
	  *dest_ptr=-1;
	  ++num_free;
	} else if(*src_ptr<occ_threshold) {
	  *dest_ptr = ++num_occupied;
	} else {
	  *dest_ptr = -2;
	  ++num_unknown;
	}
      }
    }
    /*
    cerr << "map made binary" << endl;
    cerr << "free cells:" << num_free << endl;
    cerr << "unknown cells:" << num_unknown << endl;
    cerr << "occupied cells:" << num_occupied <<  endl;
    */
  }

  void indices2distances(FloatImage& distances,
			 const IntImage& indices,
			 float resolution,
			 float max_distance) {

    unsigned int int_max_distance  = max_distance/resolution;
    DistanceMapPathSearch dmap_calculator;
    PathMap distance_map;
    dmap_calculator.setMaxDistance(int_max_distance);
    dmap_calculator.setIndicesImage(indices);
    dmap_calculator.setOutputPathMap(distance_map);
    dmap_calculator.init();
    dmap_calculator.compute();
    distances=dmap_calculator.distanceImage() * resolution;

  }


  void distances2cost(FloatImage& dest,
		      const FloatImage& src,
		      float robot_radius,
		      float safety_region,
		      float min_cost,
		      float max_cost) {
    int rows=src.rows;
    int cols=src.cols;
    dest.create(rows, cols);
    dest=0;
    int num_occupied=0;
    int num_free=0;
    int num_unknown=0;
    float slope=(max_cost-min_cost)/(robot_radius-safety_region);
    float offset=max_cost-slope*robot_radius;

    /*
    cerr << "robot_radius: " << robot_radius << endl;
    cerr << "safety_region: " << safety_region << endl;
    cerr << "min_cost: " << min_cost << endl;
    cerr << "max_cost: " << max_cost << endl;

    cerr << "slope: " << slope << endl;
    cerr << "offset: " << max_cost << endl;
    */
    
    for (int r=0; r<rows; ++r){
      float* dest_ptr=dest.ptr<float>(r);
      const float* src_ptr=src.ptr<const float>(r);
      for (int c=0; c<cols; ++c, ++dest_ptr, ++ src_ptr){
	float distance=*src_ptr;
	float& cost = *dest_ptr;
	if (distance<robot_radius){
	  cost=max_cost;
	  continue;
	}
	if (distance<safety_region){
	  cost=slope*distance+offset;
	  continue;
	}
	cost=min_cost;
      }
    }
  }

}
