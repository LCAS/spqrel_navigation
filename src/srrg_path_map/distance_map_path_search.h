#include <stdexcept>
#include "base_path_search.h"

namespace srrg_core {

  class DistanceMapPathSearch: public BasePathSearch {
  public:
    DistanceMapPathSearch();
    
    //! 
    inline int maxDistance() const {return _max_distance;}

    inline void setMaxDistance(int max_distance_) {
      _max_distance=max_distance_;
      _max_squared_distance = _max_distance*_max_distance;
    }

    //! @param indexImage: this is the input. Each occupied cell should have a unique value >-1. This value is used to refer to the closest point in the imap
    inline void setIndicesImage(const IntImage& indices_image) {_indices_image=&indices_image;}


    virtual void init() override; //< call this once after setting indices image
    
    //! computes a distance map from an int image. The cells in the image having a value >-1 are considered as occupied.
    //! it requires a reference index image to be set, otherwise it asserts
    //! the distance map is stored in the outputPathMap passed as argument from the base class
    //! each cell of the distance map points to the closest occupied cell
    //! the distances are set to the squared distances in pixels
    //! in addition, it computes the distance image in an internal member variable
    //! accessible through the distanceImage() method
    //! and the indices_map, that is an int image where each cell contains the index of the closest cell

    //! can be called on an already computed distance map
    //! to refresh it by adding new obstacle points
    //! after this, call compute() to update the distance map
    void setPoints(const Vector2iVector& points, int start_index=-1);
    
    virtual bool compute() override;

    inline const FloatImage& distanceImage() const {return _distance_image;}

    inline const IntImage& indicesMap() const {return _indices_map;}

    inline int maxIndex() const {return _max_index;}
  protected:
    void fillQueueFromImage(); // fills the expansion queue from indices images and updates max_index
    
    
    int _max_distance; // in pixels, maximum expansion of dmap
    float _max_squared_distance; // in pixels_square
    const IntImage* _indices_image;
    IntImage _indices_map;
    FloatImage _distance_image;
    PathMapCellQueue _queue;
    int _max_index;
  };

}
