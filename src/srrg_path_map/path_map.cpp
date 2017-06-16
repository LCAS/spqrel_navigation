#include "path_map.h"
#include <limits>
#include <deque>
#include <queue>
#include <set>
#include <vector>

namespace srrg_core {

  using namespace std;

  void PathMap::fill(float distance, float cost) {
    for (size_t r=0; r<rows(); ++r){
      PathMapCell* cell_ptr=_row_ptrs[r];
      for (size_t c=0; c<cols(); ++c, ++cell_ptr){
	cell_ptr->r=r;
	cell_ptr->c=c;
	cell_ptr->parent=0;
	cell_ptr->distance=distance;
	cell_ptr->cost=cost;
      }
    }
  }

  void PathMap::toImage(UnsignedCharImage& img) const {
    img.create(rows(), cols());

    float mdist = 0;
    for (int r=0; r<rows(); r++)
      for (int c=0; c<cols(); c++){
	const PathMapCell& cell = (*this)(r,c);
	if (cell.distance == std::numeric_limits<float>::max())
	  continue;
	mdist = (mdist < cell.distance) ?  cell.distance : mdist;
      }
    mdist = std::sqrt(mdist)
      ;
    // cerr << "mdist=" << mdist;
    for (int r=0; r<rows(); r++)
      for (int c=0; c<cols(); c++){
	const PathMapCell& cell = (*this)(r,c);
	float ndist = 127 * std::sqrt(cell.distance)/mdist;
	int v = 127-ndist;
	img.at<unsigned char>(r,c) = (unsigned char) v;
      }
  }


}
