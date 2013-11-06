#ifndef __SOTH_SUB_MATRIX_H__
#define __SOTH_SUB_MATRIX_H__


/*DEBUG*/#include <iostream>

/* The Eigen::MatrixBase derivations have to be done in the
 * Eigen namespace. */
namespace Eigen
{
  enum
    {
      RowPermutation,
      ColPermutation,
      RowAndColPermutation
    };

  template <typename MatrixType>
  struct ei_compute_lvalue_bit
  {
    enum {ret = (MatrixType::Flags & (DirectAccessBit | LvalueBit))?LvalueBit:0};
  };

  template<typename MatrixType, int PermutationType, bool IsSub>
  class SubMatrix;

  namespace internal {
    template<typename MatrixType, int PermutationType, bool IsSub>
    struct traits<SubMatrix<MatrixType, PermutationType, IsSub> >
      : traits<MatrixType>
    {
      typedef typename nested<MatrixType>::type MatrixTypeNested;
      typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
      typedef typename MatrixType::StorageKind StorageKind;
      enum {
	RowsAtCompileTime = (PermutationType==ColPermutation) ? (MatrixType::RowsAtCompileTime) : Dynamic,
	ColsAtCompileTime = (PermutationType==RowPermutation) ? (MatrixType::ColsAtCompileTime) : Dynamic,
	MaxRowsAtCompileTime = (IsSub ? MatrixType::MaxRowsAtCompileTime : Dynamic),
	MaxColsAtCompileTime = (IsSub ? MatrixType::MaxColsAtCompileTime : Dynamic),
	Flags = (_MatrixTypeNested::Flags & HereditaryBits) | ei_compute_lvalue_bit<_MatrixTypeNested>::ret,
	CoeffReadCost = _MatrixTypeNested::CoeffReadCost //Todo : check that
      };
    };
  }


  /* --- HELPERS ------------------------------------------------------------ */
  /* --- HELPERS ------------------------------------------------------------ */
  /* --- HELPERS ------------------------------------------------------------ */
  template<typename VectorType>
  struct ei_range_helper
  {
    typedef typename VectorType::Index Index;

    static VectorType generate(Index low, Index high)
    {
      Index size = high-low;
      if (size>1)
	return VectorType::LinSpaced(size, low, high-1);
      else if (size==0)
	return VectorType();
      else
	return VectorType::Constant(1,low);
    }
  };

  template<typename MatrixType,
	   int PermutationType,
	   int rowsAtCompileTime=MatrixType::RowsAtCompileTime,
	   int colsAtCompileTime=MatrixType::ColsAtCompileTime>
  struct ei_submatrix_index_helper{};

  template<typename MatrixType, int rowsAtCompileTime, int colsAtCompileTime>
  struct ei_submatrix_index_helper<MatrixType, RowPermutation, rowsAtCompileTime, colsAtCompileTime>
  {
    typedef typename MatrixType::Index Index;
    static Index index(Index row, Index /*col*/) {return row;}
  };

  template<typename MatrixType, int rowsAtCompileTime, int colsAtCompileTime>
  struct ei_submatrix_index_helper<MatrixType, ColPermutation, rowsAtCompileTime, colsAtCompileTime>
  {
    typedef typename MatrixType::Index Index;
    static Index index(Index /*row*/, Index col) {return col;}
  };

  template<typename MatrixType, int colsAtCompileTime>
  struct ei_submatrix_index_helper<MatrixType, RowAndColPermutation, 1, colsAtCompileTime>
  {
    typedef typename MatrixType::Index Index;
    static Index index(Index /*row*/, Index col) {return col;}
  };

  template<typename MatrixType, int rowsAtCompileTime>
  struct ei_submatrix_index_helper<MatrixType, RowPermutation, rowsAtCompileTime, 1>
  {
    typedef typename MatrixType::Index Index;
    static Index index(Index row, Index /*col*/) {return row;}
  };

  /* --- CONTAINTER --------------------------------------------------------- */
  /* --- CONTAINTER --------------------------------------------------------- */
  /* --- CONTAINTER --------------------------------------------------------- */

  // TODO: Void containter is not useful anymore, neither is ei_choose_container.
  // Remove them, and propagate the changes in submatrix.
  template<typename MatrixType>
  class VoidContainer
  {
  protected:
    VoidContainer(const MatrixType& m) {}
  };

  template<typename MatrixType>
  class MatrixContainer
  {
  protected:
  public:
    MatrixContainer(const MatrixType& m) : m_matrix(m) {}
    const typename MatrixType::Nested m_matrix;
  };

  template<typename MatrixType, int PermutationType>
  struct ei_choose_container_impl
  {
    typedef MatrixContainer<MatrixType> type;
    //  typedef VoidContainer<MatrixType> type;
  };

  template<typename MatrixType>
  struct ei_choose_container_impl<MatrixType, RowAndColPermutation>
  {
    typedef MatrixContainer<MatrixType> type;
  };


  /* --- ROW INDEX ---------------------------------------------------------- */
  /* --- ROW INDEX ---------------------------------------------------------- */
  /* --- ROW INDEX ---------------------------------------------------------- */

  template<typename MatrixType, bool IsSub=true>
  class NoRowSelectionImpl // : public MatrixContainer<MatrixType>
  {
  public:
    typedef typename MatrixType::Index Index;
    typedef VectorXi RowIndices;

  protected:
    NoRowSelectionImpl(const MatrixType& m, bool /*defaultPermutation*/ )
      : m_contain(m) {}
    NoRowSelectionImpl(const MatrixType& m, const RowIndices indices)
      : m_contain(m) {}
    NoRowSelectionImpl(const MatrixType& m, RowIndices* indices)
      : m_contain(m) {}
    MatrixContainer<MatrixType> m_contain;

  public:
    Index rowIndex(Index i) const {return i;}
    Index rows() const {return m_contain.m_matrix.rows();}
  };

  template <typename MatrixType, bool IsSub=true>
  class RowSelectionImpl
  {
  public:
    typedef typename MatrixType::Index Index;
    typedef VectorXi RowIndices;

    RowSelectionImpl(const MatrixType& m, bool defaultPermutation = false)
      : m_contain(m),owned_rowIndices(),rowIndices( owned_rowIndices )
    {
      if( defaultPermutation ) setRowRange(0,m.rows());
      else owned_rowIndices.resize(0);
    }

    /* No copy of the indices, but a storage of the reference. */
    RowSelectionImpl(const MatrixType& m,const RowIndices indices)
      : m_contain(m),owned_rowIndices(indices),rowIndices(owned_rowIndices )
    { }
    /* No copy of the indices, but a storage of the reference. */
    RowSelectionImpl(const MatrixType& m,RowIndices* indices)
      : m_contain(m),owned_rowIndices(),rowIndices( *indices )
    { }

     RowSelectionImpl(const RowSelectionImpl& clone )
      : m_contain(clone.m_contain),owned_rowIndices(clone.owned_rowIndices)
      , rowIndices( clone.rowIndicesOwned()?owned_rowIndices:clone.rowIndices )
    {}
   /* --- Kernel implementation --- */
    Index rowIndex(Index i) const
    {
      eigen_assert( inIdxRange(i) );
      eigen_assert( inMRange(rowIndices[i]) );
      return rowIndices[i];
    }
    Index rows() const {return rowIndices.size();}
    const RowIndices& getRowIndices() const { return rowIndices; }
    RowIndices& getRowIndices() { return rowIndices; }
    const Index& getRowIndices(Index i) const { return rowIndices[i]; }
    Index& getRowIndices(Index i) { return rowIndices[i]; }
    bool rowIndicesOwned() const { return &rowIndices==&owned_rowIndices; }

    /* --- Basic setters --- */
    void setRowIndices(const RowIndices& indices)
    {
      //eigen_assert(!IsSub || indices.size() < m_matrix.rows());
      rowIndices = indices;
    }
    void setRowRange(Index start, Index end)
    {
      eigen_assert( 0<=start ); eigen_assert( inMRange(std::max<long>(0,end-1)) );
      eigen_assert( start<=end );
      rowIndices = ei_range_helper<RowIndices>::generate(start, end);
    }
    void permuteRows(Index i, Index j)
    {
      eigen_assert(i>=0 && i<rows());
      eigen_assert(j>=0 && j<rows());
      Index tmp = rowIndices[i];
      rowIndices[i] = rowIndices[j];
      rowIndices[j] = tmp;
    }
    void pushRowFront(Index index)
    {
      eigen_assert( inMRange(index) );
      const Index s = rows();
      rowIndices.conservativeResize(s+1);
      /* Cannot use block for this operation. */
      for (Index i=s; i>0; --i) {rowIndices[i]=rowIndices[i-1];}
      rowIndices[0] = index;
    }
    void pushRowBack(Index index)
    {
      eigen_assert( inMRange(index) );
      const Index s = rows();
      rowIndices.conservativeResize(s+1);
      rowIndices[s] = index;
    }
    Index removeRow(Index index)
    {
      assert(index<rows());
      const Index res = rowIndices[index];
      const Index s = rows()-index-1;
      rowIndices.segment(index,s) = rowIndices.tail(s);
      rowIndices.conservativeResize(rows()-1);
      return res;
    }
    Index popRowBack()
    {
      assert(rows()>0);
      const Index res = rowIndices[rows()-1];
      rowIndices.conservativeResize(rows()-1);
      return res;
    }
    Index popRowFront()
    {
      assert(rows()>0);
      return removeRow(0);
    }

  private:
    MatrixContainer<MatrixType> m_contain;
    RowIndices owned_rowIndices;
    RowIndices & rowIndices;
    bool inMRange( Index i ) const { return 0<=i && i<m_contain.m_matrix.rows(); }
    bool inIdxRange( Index i ) const { return 0<=i && i<rows(); }
  };


  template<typename MatrixType, int PermutationType, bool IsSub>
  struct ei_choose_row_impl
  {
    typedef RowSelectionImpl<MatrixType, IsSub> type;
  };

  template<typename MatrixType, bool IsSub>
  struct ei_choose_row_impl<MatrixType, ColPermutation, IsSub>
  {
    typedef NoRowSelectionImpl<MatrixType, IsSub> type;
  };

  /* --- COL INDEX ---------------------------------------------------------- */
  /* --- COL INDEX ---------------------------------------------------------- */
  /* --- COL INDEX ---------------------------------------------------------- */

  template<typename MatrixType, bool IsSub=true>
  class NoColSelectionImpl // : public MatrixContainer<MatrixType>
  {
  public:
    typedef typename MatrixType::Index Index;
    typedef VectorXi ColIndices;
    
  protected:
    NoColSelectionImpl(const MatrixType& m, bool /*defaultPermutation*/)
      : m_contain(m){}
    NoColSelectionImpl(const MatrixType& m, const ColIndices & /* indices*/)
      : m_contain(m){}
    NoColSelectionImpl(const MatrixType& m, ColIndices* /*indices*/ )
      : m_contain(m){}
    MatrixContainer<MatrixType> m_contain;

  public:
    Index colIndex(Index i) const {return i;}
    Index cols() const {return m_contain.m_matrix.cols();}
  };

  template <typename MatrixType, bool IsSub=true>
  class ColSelectionImpl
  {
  public:
    typedef typename MatrixType::Index Index;
    typedef VectorXi ColIndices;

    ColSelectionImpl(const MatrixType& m, bool defaultPermutation)
      : m_contain(m),owned_colIndices(),colIndices( owned_colIndices )
    {
      if( defaultPermutation ) setColRange(0,m.cols());
    }

    ColSelectionImpl(const MatrixType& m,const ColIndices & indices)
      : m_contain(m),owned_colIndices(indices),colIndices( owned_colIndices )
    {}

    /* No copy of the indices, but a storage of the reference. */
    ColSelectionImpl(const MatrixType& m,ColIndices* indices)
      : m_contain(m),owned_colIndices(),colIndices( *indices )
    {}

    ColSelectionImpl(const ColSelectionImpl& clone )
      : m_contain(clone.m_contain),owned_colIndices(clone.owned_colIndices)
      , colIndices( clone.colIndicesOwned()?owned_colIndices:clone.colIndices )
    {}

    /* --- Kernel implementation --- */
    Index colIndex(Index i) const
    {
      eigen_assert( inIdxRange(i) );
      eigen_assert( inMRange(colIndices[i]) );
      return colIndices[i];
    }
    Index cols() const {return colIndices.size();}
    const ColIndices& getColIndices() const { return colIndices; }
    const Index& getColIndices(Index i) const { return colIndices[i]; }
    ColIndices& getColIndices() { return colIndices; }
    Index& getColIndices(Index i) { return colIndices[i]; }
    bool colIndicesOwned() const { return &colIndices==&owned_colIndices; }

    /* --- Basic setters --- */
    void setColIndices(const ColIndices& indices)
    {
      /* TODO eigen_assert extrema of indices. */
      colIndices = indices;
    }
    void setColRange(Index start, Index end)
    {
      eigen_assert( 0<=start ); eigen_assert( inMRange(std::max<long>(0,end-1)) );
      eigen_assert( start<=end );
      colIndices = ei_range_helper<ColIndices>::generate(start, end);
    }
    void permuteCols(Index i, Index j)
    {
      eigen_assert(i>=0 && i<cols());
      eigen_assert(j>=0 && j<cols());
      Index tmp = colIndices[i];
      colIndices[i] = colIndices[j];
      colIndices[j] = tmp;
    }
    void pushColFront(Index index)
    {
      eigen_assert( inMRange(index) );
      const Index s = cols();
      colIndices.conservativeResize(s+1);
      for (Index i=s; i>0; --i) {colIndices[i]=colIndices[i-1];}
      colIndices[0] = index;
    }
    void pushColBack(Index index)
    {
      eigen_assert( inMRange(index) );
      const Index s = cols();
      colIndices.conservativeResize(s+1);
      colIndices[s] = index;
    }
    Index removeCol(Index index)
    {
      assert(index<cols());
      const Index res = colIndices[index];
      const Index s = cols()-index-1;
      colIndices.segment(index,s) = colIndices.tail(s);
      colIndices.conservativeResize(cols()-1);
      return res;
    }
    Index popColBack()
    {
      assert(cols()>0);
      const Index res = colIndices[cols()-1];
      colIndices.conservativeResize(cols()-1);
      return res;
    }
    Index popColFront()
    {
      assert(cols()>0);
      return removeCol(0);
    }

  private:
    MatrixContainer<MatrixType> m_contain;
    ColIndices owned_colIndices;
    ColIndices & colIndices;
    bool inMRange( Index i ) const { return 0<=i && i<m_contain.m_matrix.cols(); }
    bool inIdxRange( Index i ) const { return 0<=i && i<cols(); }
  };


  template<typename MatrixType, int PermutationType, bool IsSub>
  struct ei_choose_col_impl
  {
    typedef ColSelectionImpl<MatrixType, IsSub> type;
  };

  template<typename MatrixType, bool IsSub>
  struct ei_choose_col_impl<MatrixType, RowPermutation, IsSub>
  {
    typedef NoColSelectionImpl<MatrixType, IsSub> type;
  };


  /* --- SUB MATRIX CLASS ----------------------------------------------------- */
  /* --- SUB MATRIX CLASS ----------------------------------------------------- */
  /* --- SUB MATRIX CLASS ----------------------------------------------------- */
  /* --- SUB MATRIX CLASS ----------------------------------------------------- */

  template <int PermutationType>
  struct ei_choose_assert_selection {};

  template<>
  struct ei_choose_assert_selection<RowPermutation>
  {
    enum {
      COL_PERMUTATION_IS_NOT_AVAILABLE
      ,YOU_SHOULD_HAVE_ONLY_ONE_SUBINDEX
    };
  };

  template<>
  struct ei_choose_assert_selection<ColPermutation>
  {
    enum {
      ROW_PERMUTATION_IS_NOT_AVAILABLE
      ,YOU_SHOULD_HAVE_ONLY_ONE_SUBINDEX
    };
  };

  template<>
  struct ei_choose_assert_selection<RowAndColPermutation>
  {
  };


  template<typename MatrixType, int PermutationType = RowAndColPermutation, bool IsSub=true>
  class SubMatrix
    : public MatrixBase<SubMatrix<MatrixType, PermutationType, IsSub> >
    , public ei_choose_container_impl<MatrixType, PermutationType>::type
    , public ei_choose_row_impl<MatrixType, PermutationType, IsSub>::type
    , public ei_choose_col_impl<MatrixType, PermutationType, IsSub>::type
    , public ei_choose_assert_selection<PermutationType>
  {
  public:

    typedef MatrixBase<SubMatrix<MatrixType, PermutationType, IsSub> > Base;
    typedef typename ei_choose_container_impl<MatrixType, PermutationType>::type MemoryBase;
    typedef typename ei_choose_row_impl<MatrixType, PermutationType, IsSub>::type RowBase;
    typedef typename ei_choose_col_impl<MatrixType, PermutationType, IsSub>::type ColBase;
    typedef typename RowBase::RowIndices RowIndices;
    typedef typename ColBase::ColIndices ColIndices;

    EIGEN_DENSE_PUBLIC_INTERFACE(SubMatrix)

    typedef VectorXi Indices;
    typedef ei_choose_assert_selection<PermutationType> assert_index;

    inline SubMatrix(MatrixType& matrix )
      : MemoryBase(matrix)
      , RowBase(matrix, false)
      , ColBase(matrix, false)
    {
    }

    inline SubMatrix(MatrixType& matrix, bool defaultPermutation )
      : MemoryBase(matrix)
      , RowBase(matrix, defaultPermutation)
      , ColBase(matrix, defaultPermutation)
    {
      assert(assert_index::YOU_SHOULD_HAVE_ONLY_ONE_SUBINDEX);
    }

    inline SubMatrix(MatrixType& matrix, bool defaultPermutationRows, bool defaultPermutationCols )
      : MemoryBase(matrix)
      , RowBase(matrix, defaultPermutationRows)
      , ColBase(matrix, defaultPermutationCols)
    {
    }

    /* --- By value --- */
    inline SubMatrix(MatrixType& matrix, const Indices & indices )
      : MemoryBase(matrix)
      , RowBase(matrix, indices)
      , ColBase(matrix, indices)
    {
      assert(assert_index::YOU_SHOULD_HAVE_ONLY_ONE_SUBINDEX);
    }
    inline SubMatrix(MatrixType& matrix, bool defaultPermutationRows, const ColIndices & indicesCols)
      : MemoryBase(matrix)
      , RowBase(matrix,defaultPermutationRows)
      , ColBase(matrix,indicesCols)
    {
    }
    inline SubMatrix(MatrixType& matrix, const Indices & indicesRows, bool defaultPermutationCols)
      : MemoryBase(matrix)
      , RowBase(matrix,indicesRows)
      , ColBase(matrix,defaultPermutationCols)
    {
    }

    inline SubMatrix(MatrixType& matrix, const Indices & indicesRows, const Indices & indicesCols)
      : MemoryBase(matrix)
      , RowBase(matrix,indicesRows)
      , ColBase(matrix,indicesCols)
    {
    }

    /* --- By reference --- */
    inline SubMatrix(MatrixType& matrix, Indices* indices )
      : MemoryBase(matrix)
      , RowBase(matrix, indices)
      , ColBase(matrix, indices)
    {
      assert(assert_index::YOU_SHOULD_HAVE_ONLY_ONE_SUBINDEX);
    }
    inline SubMatrix(MatrixType& matrix, bool defaultPermutationRows, const Indices* indicesCols)
      : MemoryBase(matrix)
      , RowBase(matrix,defaultPermutationRows)
      , ColBase(matrix,indicesCols)
    {
    }
    inline SubMatrix(MatrixType& matrix, const Indices* indicesRows, bool defaultPermutationCols)
      : MemoryBase(matrix)
      , RowBase(matrix,indicesRows)
      , ColBase(matrix,defaultPermutationCols)
    {
    }

    inline SubMatrix(MatrixType& matrix, Indices* indicesRows, Indices* indicesCols)
      : MemoryBase(matrix)
      , RowBase(matrix,indicesRows)
      , ColBase(matrix,indicesCols)
    {
    }


    /* --- Matrix base interface --- */
    using RowBase::rows;
    using ColBase::cols;

    EIGEN_INHERIT_ASSIGNMENT_OPERATORS(SubMatrix)

    inline Scalar& coeffRef(Index row, Index col)
    {
      return MemoryBase::m_matrix.const_cast_derived().coeffRef(this->rowIndex(row), this->colIndex(col));
    }

    inline Scalar& coeffRef(Index index)
    {
      return MemoryBase::m_matrix.const_cast_derived()
	.coeffRef(ei_submatrix_index_helper<MatrixType, PermutationType>::index(this->rowIndex(index), this->colIndex(index)));
    }

    inline const CoeffReturnType coeff(Index row, Index col) const
    {
      return MemoryBase::m_matrix.coeff(this->rowIndex(row), this->colIndex(col));
    }
    inline const CoeffReturnType coeff(Index index) const
    {
      return MemoryBase::m_matrix.const_cast_derived()
	.coeffRef(ei_submatrix_index_helper<MatrixType, PermutationType>::index(this->rowIndex(index), this->colIndex(index)));
    }
  };



  template<typename MatrixType>
  struct ColContainer
  {
    typedef typename MatrixType::ColXpr NestedType;
    NestedType nested;
    typedef typename MatrixType::Index Index;
    ColContainer( MatrixType& m,Index col ) : nested(m.col(col)) {}
  };

  template<typename MatrixType,bool IsSub=true>
  class SubCol
    :protected ColContainer<MatrixType>
    ,public SubMatrix< typename ColContainer<MatrixType>::NestedType,RowPermutation,IsSub >
  {
  public:
    typedef typename MatrixType::Index Index;
    typedef typename ColContainer<MatrixType>::NestedType NestedType;
    typedef SubMatrix< NestedType,RowPermutation,IsSub > SubMatrixBase;
    typedef typename SubMatrixBase::Indices Indices;
    typedef ColContainer<MatrixType> ContainerBase;
    using ContainerBase::nested;

    SubCol( MatrixType& m, const Indices& indicesRows, Index c )
      : ContainerBase(m,c), SubMatrixBase( nested,indicesRows )
    {}

  protected:

  };


  typedef SubMatrix<MatrixXd> SubMatrixXd;
  typedef SubMatrix<VectorXd,RowPermutation> SubVectorXd;
  typedef const SubMatrixXd const_SubMatrixXd;
  typedef const SubVectorXd const_SubVectorXd;


  /* --- STACK -------------------------------------------------------------- */
  /* --- STACK -------------------------------------------------------------- */
  /* --- STACK -------------------------------------------------------------- */
  template<typename MatrixType1,typename MatrixType2>
  class StackMatrix;

  namespace internal
  {
    template<typename MatrixType1,typename MatrixType2>
    struct traits< StackMatrix<MatrixType1,MatrixType2> >
      : traits<MatrixType1>
    {
      typedef typename nested<MatrixType1>::type MatrixTypeNested;
      typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
      typedef typename MatrixType1::StorageKind StorageKind;
      enum {
	RowsAtCompileTime = MatrixType1::RowsAtCompileTime,
	ColsAtCompileTime = MatrixType1::ColsAtCompileTime,
	MaxRowsAtCompileTime = MatrixType1::MaxRowsAtCompileTime,
	MaxColsAtCompileTime = MatrixType1::MaxColsAtCompileTime,
	Flags = (_MatrixTypeNested::Flags & HereditaryBits) | ei_compute_lvalue_bit<_MatrixTypeNested>::ret,
	CoeffReadCost = _MatrixTypeNested::CoeffReadCost //Todo : check that
      };
    };
  }



  template<typename MatrixType1,typename MatrixType2>
  class StackMatrix
    : public MatrixBase< StackMatrix<MatrixType1,MatrixType2> >
  {
  public:

    typedef MatrixBase< StackMatrix<MatrixType1,MatrixType2> > Base;

    EIGEN_DENSE_PUBLIC_INTERFACE(StackMatrix)

    typedef MatrixBase< MatrixType1 > Base1;
    typedef MatrixBase< MatrixType2 > Base2;

    inline StackMatrix(Base1& m1, Base2& m2 )
      : m1(m1),m2(m2)
    {
      assert( m1.cols() == m2.cols() );
    }

    /* --- Matrix base interface --- */
    EIGEN_INHERIT_ASSIGNMENT_OPERATORS(StackMatrix)

    inline Index cols() const { assert( m1.cols() == m2.cols() ); return m1.cols(); }
    inline Index rows() const { return m1.rows() + m2.rows(); }
    inline Index innerStride() const {  return 1;}
    //inline void innerStride() const { }

    inline Scalar& coeffRef(Index row, Index col)
    {
      const Index r1 = m1.rows();
      if( row<r1 ) return m1(row,col); else return m2(row-r1,col);
    }

    inline Scalar& coeffRef(Index index)
    {
      const Index r1 = m1.rows();
      if( index<r1 ) return m1(index); else return m2(index-r1);
    }
    inline const CoeffReturnType coeff(Index row, Index col) const
    {
      const Index r1 = m1.rows();
      if( row<r1 ) return m1(row,col); else return m2(row-r1,col);
    }
    inline const CoeffReturnType coeff(Index index) const
    {
      const Index r1 = m1.rows();
      if( index<r1 ) return m1(index); else return m2(index-r1);
    }

  protected:
    Base1& m1;
    Base2& m2;

  };



} // namespace soth

#endif // __SOTH_SUB_MATRIX_H__

