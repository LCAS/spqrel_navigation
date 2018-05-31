#include <cstring>
#include <stdexcept>

namespace srrg_core {
  
	
  template <typename CellType_, typename AllocatorType_>
  Vector2D<CellType_, AllocatorType_>::Vector2D(std::size_t rows_, std::size_t cols_) {
    _cols=0;
    resize(rows_, cols_);
  }
  
  
  template <typename CellType_, typename AllocatorType_>
  Vector2D<CellType_, AllocatorType_>::Vector2D(const Vector2D& other) {
    copy(other);
  }

  template <typename CellType_, typename AllocatorType_>
  Vector2D<CellType_, AllocatorType_>&
  Vector2D<CellType_, AllocatorType_>::operator = (const Vector2D& other) {
    copy(other);
    return *this;
  }
  
  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::fill (const CellType_& value) {
    std::fill(_data.begin(), _data.end(), value);
  }

  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::updateEightNeighborOffsets() {
    if (rows()<2 || cols()<2) {
      memset(_eight_neighbors_offsets, 0, 8 * sizeof(int));
      return;
    }
    int k=0;
    for (int r=-1; r<=1; ++r) {
	for (int c=-1; c<=1; ++c) {
	  if (!r && !c)
	    continue;
	  _eight_neighbors_offsets[k] = _cols*r+c;
	  ++k;
	}
      }
  }


  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::resize(std::size_t rows_, std::size_t cols_){
    // if size is ok, do nothing
    if (rows_==_row_ptrs.size() && _cols==cols_)
      return;
    std::size_t num_elements=rows_*cols_;
    if (! num_elements)
      clear();
    _cols=cols_;
    _row_ptrs.resize(rows_);
    _data.resize(num_elements);
    reindex();
  }
  
     
  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::clear() {
    _cols=0;
    _row_ptrs.clear();
    _data.clear();
  }

  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::reindex() {
    CellType* row_ptr=&_data[0];
    for (std::size_t r=0; r<rows(); r++){
      _row_ptrs[r]=row_ptr;
      row_ptr+=_cols;
    }
    updateEightNeighborOffsets();
  }

  template <typename CellType_, typename AllocatorType_>
  void
  Vector2D<CellType_, AllocatorType_>::copy(const Vector2D& other) {
    if (!other._data.size()){
      clear();
      return;
    }
    _cols=other._cols;
    _data=other._data;
    _row_ptrs.resize(other._row_ptrs.size());
    reindex();
  }

}// end namespace
