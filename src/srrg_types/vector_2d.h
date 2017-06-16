#pragma once
#include <vector>
#include <stdexcept>
#include <cstring>

namespace srrg_core{
  /**
     Implements a simplistic 2D array class, that supports copy and resize
     Order is row major. An array of row pointers is dynamically constructed
     for fast scattered access
     @param CellType_ the type of the cell
     @param AllocatorType the allocator
   */

  template <typename CellType_, typename AllocatorType_=std::allocator< CellType_ > >
  class Vector2D{
  public:
    typedef CellType_ CellType;
    typedef AllocatorType_ AllocatorType;
    
    Vector2D(std::size_t rows_=0, std::size_t cols_=0);

    Vector2D(const Vector2D& other);

    Vector2D<CellType, AllocatorType_>& operator = (const Vector2D& other);

    inline std::size_t rows() const {return _row_ptrs.size();}

    inline std::size_t cols() const {return _cols;}

    inline std::size_t capacity() const {return _data.capacity(); }

    inline void reserve(int num_elements) { _data.reserve(num_elements); }
    
    inline const CellType* rowPtr(int row) const {return _row_ptrs[row];}

    inline CellType* rowPtr(int row) {return _row_ptrs[row];}
    
    inline const std::vector<CellType>& data() const  {return _data;}
    
    inline std::vector<CellType>& data()  {return _data;}
    
    inline bool onBorder(std::size_t row, std::size_t col) const {
      return row==0 || col==0 || row==rows()-1 || col==_cols-1;
    }

    inline bool inside(std::size_t row, std::size_t col) const {
      return row>=0 && col>=0 &&row<rows() && col<_cols;
    }

    inline CellType& at(std::size_t row, std::size_t col) {
      if (col>=_cols)
	throw std::out_of_range("columns");
      return _row_ptrs.at(row)[col];
    }

    inline const CellType& at(std::size_t row, std::size_t col) const {
      if (col>=_cols)
	throw std::out_of_range("columns");
      return _row_ptrs.at(row)[col];
    }

    inline const CellType& operator()(std::size_t row, std::size_t col) const {
      return _row_ptrs[row][col];
    }

    inline CellType& operator()(std::size_t row, std::size_t col) {
      return _row_ptrs[row][col];
    }

    inline const int* eightNeighborOffsets() const {return _eight_neighbors_offsets;}
    

    void resize(std::size_t rows_, std::size_t cols_);
     
    void clear();
    
    void fill (const CellType_& value);
    
  protected:
    void reindex();
    void copy(const Vector2D& other);
    void updateEightNeighborOffsets();
    
    std::size_t _cols;
    std::vector<CellType*> _row_ptrs;
    std::vector< CellType, AllocatorType > _data;

    int _eight_neighbors_offsets[8];
  };
  
}// end namespace

#include "vector_2d.hpp"
