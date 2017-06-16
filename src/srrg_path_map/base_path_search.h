#pragma once
#include <deque>
#include <queue>
#include "path_map.h"

namespace srrg_core {
  class BasePathSearch{
  public:
    BasePathSearch();
    virtual ~BasePathSearch() = 0;
    virtual void setup(); // to set the parameters after construction

    inline void setOutputPathMap(PathMap& output) {_output_path_map=&output;}
    inline const PathMap& outputPathMap() const {return *_output_path_map;}
    inline  PathMap& outputPathMap() {return *_output_path_map;}
    
    virtual void init();        // to be called before compute when all parameters are in place
    virtual bool compute()=0; // to do the actual calculation. Return false on unreachable goal. Output written on pathmap;

    inline const int numOperations() const {return _num_operations;}
  protected:
    PathMap *_output_path_map;
    size_t _num_operations;
  };

    //! Queue entry to compute the distance map from an IntImage
  //! It is exposed here to support the construction of different sorts of visit
  //! algorithms, such as Dijkstra or A*

  struct QEntry{
    QEntry(PathMapCell* c=0, float d=std::numeric_limits<float>::max()) {
      cell = c;
      distance = d;
    }

    //! comparison operator, returns the closest cell
    inline bool operator < (const QEntry& e) const {
      return e.distance < distance ;
    }

    float distance;
    PathMapCell* cell;
  };
  
  //! priority queue of PathMapCells, supports several algorithms
  //! it is a sorted container that keeps the entries ordered bu the distance
  struct PathMapCellQueue : public std::priority_queue<QEntry> {
    typedef typename std::priority_queue<QEntry>::size_type size_type;
    PathMapCellQueue(size_type capacity = 0) { reserve(capacity); };
    inline void reserve(size_type capacity) { this->c.reserve(capacity); } 
    inline size_type capacity() const { return this->c.capacity(); } 
    //! returns the top element of the queue;
    inline PathMapCell* top() { return std::priority_queue<QEntry>::top().cell;}
    //! pushes an element in the priority queue
    inline void push(PathMapCell* c) { return std::priority_queue<QEntry>::push(QEntry(c, c->distance));}
  };

}
