#include <stdexcept>
#include "base_path_search.h"

namespace srrg_core {

  class DijkstraPathSearch: public BasePathSearch {
  public:
    DijkstraPathSearch();
    
    //! 
    inline float maxCost() const {return _max_cost;}

    inline void setMaxCost(float max_cost_) {
      _max_cost=max_cost_;
    }

    inline float cellTraversalCost() const { return _cell_traversal_cost; }

    inline void setCellTraversalCost(float cell_traversal_cost) {
      _cell_traversal_cost = cell_traversal_cost;
      _cell_traversal_cost_diagonal = std::sqrt(2.0f) *_cell_traversal_cost;
    }

    //! @param sets a map that contains the cost of being in a particular cell. This is the input.
    inline void setCostMap(const FloatImage& cost_map) {_cost_map=&cost_map;}

    inline Vector2iVector& goals() {return _goals;}

    virtual bool compute() override;

  protected:
    float _max_cost; // in pixels, maximum cost for a cell(used to avoid obstacles)
    float _cell_traversal_cost, _cell_traversal_cost_diagonal; // 
    const FloatImage* _cost_map;
    Vector2iVector _goals;
    IntImage _indices_map;
    FloatImage _distance_image;
  };

}
