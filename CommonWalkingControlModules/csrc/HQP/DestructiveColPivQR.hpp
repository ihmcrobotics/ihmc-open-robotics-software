#ifndef EIGEN_DESTRUCTIVE_COLPIV_QR_H
#define EIGEN_DESTRUCTIVE_COLPIV_QR_H


namespace Eigen
{

/** This class is a modification of Eigen's ColPivHouseholdeQR to perform a rank-revealing QR with column pivoting
  * MP = QR with R*P' directly stored in the input matrix M and the householder vectors essential parts stored in the
  * column of a different matrix given by the user.
  */
template<typename _MatrixType, typename _HouseholderStorageType= typename internal::plain_matrix_type<_MatrixType>::type>
class DestructiveColPivQR
{
  public:

    typedef _MatrixType MatrixType;
    enum {
      RowsAtCompileTime = MatrixType::RowsAtCompileTime,
      ColsAtCompileTime = MatrixType::ColsAtCompileTime,
      //Options = MatrixType::Options,
      MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
      MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
    };
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::RealScalar RealScalar;
    typedef typename MatrixType::Index Index;
    //typedef Matrix<Scalar, RowsAtCompileTime, RowsAtCompileTime, Options, MaxRowsAtCompileTime, MaxRowsAtCompileTime> MatrixQType;
    typedef _HouseholderStorageType  MatrixQType;
    //typedef typename internal::plain_diag_type<MatrixType>::type HCoeffsType;
    typedef Diagonal<MatrixQType,0> HCoeffsType;
    typedef PermutationMatrix<ColsAtCompileTime, MaxColsAtCompileTime> PermutationType;
    typedef typename internal::plain_row_type<MatrixType, Index>::type IntRowVectorType;
    typedef typename internal::plain_row_type<MatrixType>::type RowVectorType;
    typedef typename internal::plain_row_type<MatrixType, RealScalar>::type RealRowVectorType;
    typedef typename HouseholderSequence<MatrixQType,HCoeffsType>::ConjugateReturnType HouseholderSequenceType;


    DestructiveColPivQR(MatrixType& matrix, MatrixQType& householderEssentialStorage, RealScalar epsilon=0.)
      : m_r(matrix),
        m_q(householderEssentialStorage),
        //m_hCoeffs(std::min(matrix.rows(),matrix.cols())),
        m_hCoeffs(householderEssentialStorage.diagonal()),
        m_colsPermutation(matrix.cols()),
        m_colsTranspositions(matrix.cols()),
        m_colsIntTranspositions(matrix.cols()),
        m_temp(matrix.cols()),
        m_colSqNorms(matrix.cols()),
        m_isInitialized(false),
        m_usePrescribedEpsilon((epsilon==0.)? false : true),
        m_prescribedEpsilon(epsilon)
    {
      eigen_assert(epsilon >= 0.);
      compute();
    }

    HouseholderSequenceType householderQ(void) const;

    /** \returns a reference to the matrix where the Householder QR decomposition is stored
      */
    const MatrixType& matrixR() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return m_r;
    }

    DestructiveColPivQR& compute(MatrixType& matrix, MatrixQType& householderEssentialStorage)
    {
      m_r = matrix;
      m_q = householderEssentialStorage;
    }

    DestructiveColPivQR& compute();

    const PermutationType& colsPermutation() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return m_colsPermutation;
    }

    /** \returns the absolute value of the determinant of the matrix of which
      * *this is the QR decomposition. It has only linear complexity
      * (that is, O(n) where n is the dimension of the square matrix)
      * as the QR decomposition has already been computed.
      *
      * \note This is only for square matrices.
      *
      * \warning a determinant can be very big or small, so for matrices
      * of large enough dimension, there is a risk of overflow/underflow.
      * One way to work around that is to use logAbsDeterminant() instead.
      *
      * \sa logAbsDeterminant(), MatrixBase::determinant()
      */
    typename MatrixType::RealScalar absDeterminant() const;

    /** \returns the natural log of the absolute value of the determinant of the matrix of which
      * *this is the QR decomposition. It has only linear complexity
      * (that is, O(n) where n is the dimension of the square matrix)
      * as the QR decomposition has already been computed.
      *
      * \note This is only for square matrices.
      *
      * \note This method is useful to work around the risk of overflow/underflow that's inherent
      * to determinant computation.
      *
      * \sa absDeterminant(), MatrixBase::determinant()
      */
    typename MatrixType::RealScalar logAbsDeterminant() const;

    /** \returns the rank of the matrix of which *this is the QR decomposition.
      *
      * \note This method has to determine which pivots should be considered nonzero.
      *       For that, it uses the threshold value that you can control by calling
      *       setThreshold(const RealScalar&).
      */
    inline Index rank() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return m_nonzero_pivots;
    }

    /** \returns the dimension of the kernel of the matrix of which *this is the QR decomposition.
      *
      * \note This method has to determine which pivots should be considered nonzero.
      *       For that, it uses the threshold value that you can control by calling
      *       setThreshold(const RealScalar&).
      */
    inline Index dimensionOfKernel() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return cols() - rank();
    }

    /** \returns true if the matrix of which *this is the QR decomposition represents an injective
      *          linear map, i.e. has trivial kernel; false otherwise.
      *
      * \note This method has to determine which pivots should be considered nonzero.
      *       For that, it uses the threshold value that you can control by calling
      *       setThreshold(const RealScalar&).
      */
    inline bool isInjective() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return rank() == cols();
    }

    /** \returns true if the matrix of which *this is the QR decomposition represents a surjective
      *          linear map; false otherwise.
      *
      * \note This method has to determine which pivots should be considered nonzero.
      *       For that, it uses the threshold value that you can control by calling
      *       setThreshold(const RealScalar&).
      */
    inline bool isSurjective() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return rank() == rows();
    }

    /** \returns true if the matrix of which *this is the QR decomposition is invertible.
      *
      * \note This method has to determine which pivots should be considered nonzero.
      *       For that, it uses the threshold value that you can control by calling
      *       setThreshold(const RealScalar&).
      */
    inline bool isInvertible() const
    {
      eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
      return isInjective() && isSurjective();
    }

    inline Index rows() const { return m_r.rows(); }
    inline Index cols() const { return m_r.cols(); }
    const HCoeffsType& hCoeffs() const { return m_hCoeffs; }

    /** Allows to prescribe a threshold to be used by certain methods, such as rank(),
      * who need to determine when pivots are to be considered nonzero. This is not used for the
      * QR decomposition itself.
      *
      * When it needs to get the threshold value, Eigen calls threshold(). By default, this
      * uses a formula to automatically determine a reasonable threshold.
      * Once you have called the present method setThreshold(const RealScalar&),
      * your value is used instead.
      *
      * \param threshold The new value to use as the threshold.
      *
      * A pivot will be considered nonzero if its absolute value is strictly greater than
      *  \f$ \vert pivot \vert \leqslant threshold \times \vert maxpivot \vert \f$
      * where maxpivot is the biggest pivot.
      *
      * If you want to come back to the default behavior, call setThreshold(Default_t)
      */
    DestructiveColPivQR& setEpsilon(const RealScalar& epsilon)
    {
      eigen_assert(epsilon > 0.);
      m_usePrescribedEpsilon = true;
      m_prescribedEpsilon = epsilon;
      return *this;
    }

    /** Allows to come back to the default behavior, letting Eigen use its default formula for
      * determining the threshold.
      *
      * You should pass the special object Eigen::Default as parameter here.
      * \code qr.setThreshold(Eigen::Default); \endcode
      *
      * See the documentation of setThreshold(const RealScalar&).
      */
    DestructiveColPivQR& setEpsilon(Default_t)
    {
      m_usePrescribedEpsilon = false;
      return *this;
    }

    /** Returns the threshold that will be used by certain methods such as rank().
      *
      * See the documentation of setThreshold(const RealScalar&).
      */
    RealScalar epsilon() const
    {
      return m_usePrescribedEpsilon ? m_prescribedEpsilon * m_r.diagonalSize()
      // this formula comes from experimenting (see "LU precision tuning" thread on the list)
      // and turns out to be identical to Higham's formula used already in LDLt.
                                      : NumTraits<Scalar>::epsilon() * m_r.diagonalSize();
    }

    /** \returns the number of nonzero pivots in the QR decomposition.
      * Here nonzero is meant in the exact sense, not in a fuzzy sense.
      * So that notion isn't really intrinsically interesting, but it is
      * still useful when implementing algorithms.
      *
      * \sa rank()
      */
    inline Index nonzeroPivots() const
    {
      eigen_assert(m_isInitialized && "LU is not initialized.");
      return m_nonzero_pivots;
    }

    /** \returns the absolute value of the biggest pivot, i.e. the biggest
      *          diagonal coefficient of U.
      */
    RealScalar maxPivot() const { return m_maxpivot; }

  protected:
    MatrixType& m_r;
    MatrixQType& m_q;
    HCoeffsType m_hCoeffs;
    PermutationType m_colsPermutation;
    IntRowVectorType m_colsTranspositions;
    IntRowVectorType m_colsIntTranspositions;
    RowVectorType m_temp;
    RealRowVectorType m_colSqNorms;
    bool m_isInitialized, m_usePrescribedEpsilon;
    RealScalar m_prescribedEpsilon, m_maxpivot;
    Index m_nonzero_pivots;
    Index m_det_pq;
};

template<typename MatrixType, typename HouseholderStrorageType>
typename MatrixType::RealScalar DestructiveColPivQR<MatrixType, HouseholderStrorageType>::absDeterminant() const
{
  eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
  eigen_assert(m_r.rows() == m_r.cols() && "You can't take the determinant of a non-square matrix!");
  return internal::abs(m_r.diagonal().prod());
}

template<typename MatrixType, typename HouseholderStrorageType>
typename MatrixType::RealScalar DestructiveColPivQR<MatrixType, HouseholderStrorageType>::logAbsDeterminant() const
{
  eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
  eigen_assert(m_r.rows() == m_r.cols() && "You can't take the determinant of a non-square matrix!");
  return m_r.diagonal().cwiseAbs().array().log().sum();
}

template<typename MatrixType, typename HouseholderStrorageType>
DestructiveColPivQR<MatrixType, HouseholderStrorageType>& DestructiveColPivQR<MatrixType, HouseholderStrorageType>::compute()
{

  Index rows = m_r.rows();
  Index cols = m_r.cols();
  Index size = m_r.diagonalSize();

  assert(m_r.diagonalSize() > 0);
  //TODO : handle this case ?

  //m_hCoeffs.resize(size);

  m_temp.resize(cols);

  m_colsTranspositions.resize(cols);
  m_colsIntTranspositions.resize(cols);
  if (cols>1)
    m_colsIntTranspositions =  IntRowVectorType::LinSpaced(cols,0, cols-1);
  else
    m_colsIntTranspositions =  IntRowVectorType::Zero(1); //TODO: treat this special case here. It just requires to check every output data is correctly initilized

  Index number_of_transpositions = 0;

  m_colSqNorms.resize(cols);
  for(Index k = 0; k < cols; ++k)
    m_colSqNorms.coeffRef(k) = m_r.col(k).squaredNorm();

  //RealScalar threshold_helper = m_colSqNorms.maxCoeff() * internal::abs2(epsilon()) /rows;
  // The threshold should be decided wrt. to the norm of ML, while this class only consider
  // the norm of L -> TODO: add an initialization of threshold by EPS*norm(L) ... is it really
  // necessary, 'cos it is time consuming.
  RealScalar threshold_helper = internal::abs2(epsilon());

  m_nonzero_pivots = size; // the generic case is that in which all pivots are nonzero (invertible case)
  m_maxpivot = RealScalar(0);

  for(Index k = 0; k < size; ++k)
  {
    // first, we look up in our table m_colSqNorms which column has the biggest squared norm
    Index biggest_col_index;
    RealScalar biggest_col_sq_norm = m_colSqNorms.tail(cols-k).maxCoeff(&biggest_col_index);
    biggest_col_index += k;

    // since our table m_colSqNorms accumulates imprecision at every step, we must now recompute
    // the actual squared norm of the selected column.
    // Note that not doing so does result in solve() sometimes returning inf/nan values
    // when running the unit test with 1000 repetitions.
    biggest_col_sq_norm = m_r.col(m_colsIntTranspositions[biggest_col_index]).tail(rows-k).squaredNorm();

    // we store that back into our table: it can't hurt to correct our table.
    m_colSqNorms.coeffRef(biggest_col_index) = biggest_col_sq_norm;

    // if the current biggest column is smaller than epsilon times the initial biggest column,
    // terminate to avoid generating nan/inf values.
    // Note that here, if we test instead for "biggest == 0", we get a failure every 1000 (or so)
    // repetitions of the unit test, with the result of solve() filled with large values of the order
    // of 1/(size*epsilon).
    if(biggest_col_sq_norm < threshold_helper * (rows-k))
    {
      m_nonzero_pivots = k;
      m_hCoeffs.tail(size-k).setZero();
      m_q.bottomRightCorner(rows-k,rows-k)
          .template triangularView<StrictlyLower>()
          .setZero();
      break;
    }

    // apply the transposition to the columns
    m_colsTranspositions.coeffRef(k) = biggest_col_index;
    if(k != biggest_col_index) {
      std::swap(m_colsIntTranspositions.coeffRef(k), m_colsIntTranspositions.coeffRef(biggest_col_index));
      //std::swap(m_colsInvTranspositions.coeffRef(m_colsTranspositions[k]), m_colsInvTranspositions.coeffRef(m_colsTranspositions[biggest_col_index]));
      //m_qr.col(k).swap(m_qr.col(biggest_col_index));
      std::swap(m_colSqNorms.coeffRef(k), m_colSqNorms.coeffRef(biggest_col_index));
      ++number_of_transpositions;
    }

    // generate the householder vector, store it below the diagonal
    RealScalar beta;
    //m_qr.col(k).tail(rows-k).makeHouseholderInPlace(m_hCoeffs.coeffRef(k), beta);
    VectorBlock<typename MatrixQType::ColXpr> colk = m_q.col(k).tail(rows-k-1);
    m_r.col(m_colsIntTranspositions[k]).tail(rows-k).makeHouseholder(colk, m_hCoeffs.coeffRef(k), beta);

    // apply the householder transformation to the diagonal coefficient
    m_r.coeffRef(k,m_colsIntTranspositions[k]) = beta;
    m_r.col(m_colsIntTranspositions[k]).tail(rows-k-1).setZero();

    // remember the maximum absolute value of diagonal coefficients
    if(internal::abs(beta) > m_maxpivot) m_maxpivot = internal::abs(beta);

    // apply the householder transformation
    for (Index l = k+1; l<cols; ++l)
      m_r.col(m_colsIntTranspositions[l]).tail(rows-k)
        .applyHouseholderOnTheLeft(m_q.col(k).tail(rows-k-1), m_hCoeffs.coeffRef(k), &m_temp.coeffRef(k+1));
    //m_r.bottomRightCorner(rows-k, cols-k-1)
        //.applyHouseholderOnTheLeft(m_q.col(k).tail(rows-k-1), m_hCoeffs.coeffRef(k), &m_temp.coeffRef(k+1));

    // update our table of squared norms of the columns
    for (Index l = k+1; l<cols; ++l)
      m_colSqNorms[l] -= m_r(k,m_colsIntTranspositions[l])*m_r(k,m_colsIntTranspositions[l]);

    // Nullify the lower-diag part of the column
    //colk.tail(rows-k-1).setZero();

    //m_colSqNorms.tail(cols-k-1) -= m_r.row(k).tail(cols-k-1).cwiseAbs2();
    //std::cout << "R=" << std::endl << m_r << std::endl;
    //std::cout << "norms=" << std::endl << m_colSqNorms << std::endl;
  }

  m_colsPermutation.setIdentity(cols);
  for(Index k = 0; k < m_nonzero_pivots; ++k)
    m_colsPermutation.applyTranspositionOnTheRight(k, m_colsTranspositions.coeff(k));

  m_det_pq = (number_of_transpositions%2) ? -1 : 1;
  m_isInitialized = true;

  return *this;
}

/** \returns the matrix Q as a sequence of householder transformations */
template<typename MatrixType, typename HouseholderStrorageType>
typename DestructiveColPivQR<MatrixType, HouseholderStrorageType>::HouseholderSequenceType DestructiveColPivQR<MatrixType, HouseholderStrorageType>
  ::householderQ() const
{
  eigen_assert(m_isInitialized && "DestructiveColPivQR is not initialized.");
  HouseholderSequenceType hh(m_q, m_hCoeffs.conjugate()); //, false, m_nonzero_pivots, 0);
  hh.setLength(m_nonzero_pivots);
  return hh;
}

} //namespace Eigen

#endif // EIGEN_DESTRUCTIVE_COLPIV_QR_H
