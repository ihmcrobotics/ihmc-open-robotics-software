#ifndef __SOTH_GIVENS__
#define __SOTH_GIVENS__

#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <soth/api.hpp>
#include <soth/SubMatrix.hpp>
#include <soth/Algebra.hpp>
#include <vector>


namespace soth
{
  using namespace Eigen;



  /* --- SINGLE ------------------------------------------------------------- */
  /* --- SINGLE ------------------------------------------------------------- */
  /* --- SINGLE ------------------------------------------------------------- */
  class Givens
  {
  public:
    typedef JacobiRotation<double> NestedType;

  public:
    SOTH_EXPORT Givens();
    SOTH_EXPORT Givens(double a, double b, int i, int j, double* z=0);

    template<typename VectorBase>
    Givens(const VectorBase & v, int i, int j);
    template<typename VectorBase>
    Givens(VectorBase & v, int i, int j, bool apply = false);

  public:
    SOTH_EXPORT void makeGivens(double a, double b, int i, int j, double* z=0);

    /* Givens to the right, ie such that G'*v (:= G.applyTransposeOnTheRight(v) )
     * is nullified. */
    template<typename VectorBase>
    void makeGivens(const VectorBase & v, int i, int j );
    template<typename VectorBase>
    void makeGivens(VectorBase & v, int i, int j, bool apply);

    /* --- Multiplication --- */
    // M := M*G.
    template<typename D> void applyThisOnTheLeft(MatrixBase<D> & M) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyThisOnTheLeft(SubMatrix<MatrixType,PermutationType,IsSub> & M) const;
    // M := M*G'.
    template<typename D> void applyTransposeOnTheLeft(MatrixBase<D> & M) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyTransposeOnTheLeft(SubMatrix<MatrixType,PermutationType,IsSub> & M) const;
    // M := G*M.
    template<typename D> void applyThisOnTheRight(MatrixBase<D> & M) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyThisOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub> & M) const;
    // M := G'*M.
    template<typename D> void applyTransposeOnTheRight(MatrixBase<D> & M) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyTransposeOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub> & M) const;

    /* --- MULTIPLICATION PARTIEL --- */
    // M := M*G.
    template<typename D>
    void applyThisOnTheLeftPartiel(MatrixBase<D> & M) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyThisOnTheLeftPartiel(SubMatrix<MatrixType,PermutationType,IsSub> & M) const;

    // M := G'*M. -- TODO: use Index instead of int
    template<typename D> void applyTransposeOnTheRight(MatrixBase<D> & M, const int size) const;
    template<typename MatrixType, int PermutationType, bool IsSub>
    void applyTransposeOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub> & M, const int size) const;
    // TODO: same partiel method for all the cases


    /* For application on lines of triangular matrices: check
     * if the plannar rotation apply on the 0s. */
    bool applicable( unsigned int size ) const
    {
      bool ci = (i<size);
      bool cj = (j<size);
      assert( (ci&&cj)||((!ci)&&(!cj)) );
      return ci&&cj;
    }

    // [ v1;v2 ] := G*[v1;v2] -- [v2 v2] := [v1 v2]*G'
    template< typename D1,typename D2 >
    void applyThisOnVectors( MatrixBase<D1> & v1, MatrixBase<D2>& v2 )
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D1>);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D2>);
      const int size = std::min( v1.size(),v2.size() );
      for( unsigned int i=0;i<size;++i )
	{
	  double & m_1 = v1(i), & m_2 = v2(i);
	  const double x1=m_1, x2=m_2;
	  m_1 =  G.c() * x1 + internal::conj(G.s()) * x2;
	  m_2 = -G.s() * x1 + internal::conj(G.c()) * x2;
	}
    }
    // [ v1;v2 ] := G'*[v1;v2] -- [v2 v2] := [v1 v2]*G
    template< typename D1,typename D2 >
    void applyTransposeOnVectors( MatrixBase<D1> & v1, MatrixBase<D2>& v2 )
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D1>);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D2>);
      const int size = std::min( v1.size(),v2.size() );
      for( unsigned int c=0;c<size;++c )
	{
	  double & m_1 = v1(c), & m_2 = v2(c);
	  const double x1=m_1, x2=m_2;
	  m_1 =  Gt.c() * x1 + internal::conj(Gt.s()) * x2;
	  m_2 = -Gt.s() * x1 + internal::conj(Gt.c()) * x2;
	}
    }



  private: public: //DEBUG
    NestedType G,Gt;
    unsigned int i;
    unsigned int j;

  public: // Transpose

    struct Transpose
    {
      explicit Transpose( const Givens * g ) : nested(g) {}
      const Givens* nested;
    };
    Transpose transpose() const { return Transpose(this); }
  };

  template<typename D>
  void operator >> (const Givens& G, MatrixBase<D> & M) { G.applyThisOnTheRight(M); }
  template<typename D>
  void operator << (MatrixBase<D> & M,const Givens& G) { G.applyThisOnTheLeft(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator >> (const Givens& G, SubMatrix<MatrixType,PermutationType,IsSub> & M)
  { G.applyThisOnTheRight(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator << (SubMatrix<MatrixType,PermutationType,IsSub> & M,const Givens& G)
  { G.applyThisOnTheLeft(M); }


  template<typename D>
  void operator >> (const Givens::Transpose& G, MatrixBase<D> & M)
  { G.nested->applyTransposeOnTheRight(M); }
  template<typename D>
  void operator << (MatrixBase<D> & M,const Givens::Transpose& G)
  { G.nested->applyTransposeOnTheLeft(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator >> (const Givens::Transpose& G,
		    SubMatrix<MatrixType,PermutationType,IsSub> & M)
  { G.nested->applyTransposeOnTheRight(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator << (SubMatrix<MatrixType,PermutationType,IsSub> & M,
		    const Givens::Transpose& G)
  { G.nested->applyTransposeOnTheLeft(M); }



  /* --- SEQUENCE------------------------------------------------------------ */
  /* --- SEQUENCE------------------------------------------------------------ */
  /* --- SEQUENCE------------------------------------------------------------ */

  class GivensSequence
  {
  public:
    SOTH_EXPORT GivensSequence& push(const Givens& g);
    void clear() { G.clear(); }
    void reserve( unsigned int ncsquare ) { G.reserve(ncsquare); }

    // M := M*G.
    template<typename Derived>
    void applyThisOnTheLeft(MatrixBase<Derived> & M) const;
    // M := M*G'.
    template<typename Derived>
    void applyTransposeOnTheLeft(MatrixBase<Derived> & M) const;
    // M := G*M.
    template<typename Derived>
    void applyThisOnTheRight(MatrixBase<Derived> & M) const;
    // M := G'*M.
    template<typename Derived>
    void applyTransposeOnTheRight(MatrixBase<Derived> & M) const;

    // M := M*G.
    template<typename Derived>
    void applyThisOnTheLeftReduced(MatrixBase<Derived> & M) const;

  private:
    std::vector<Givens> G;

  public: // Transpose

    struct Transpose
    {
      explicit Transpose( const GivensSequence * g ) : nested(g) {}
      const GivensSequence* nested;
    };
    Transpose transpose() const { return Transpose(this); }
  };


  template<typename D>
  void operator >> (const GivensSequence& G, MatrixBase<D> & M) { G.applyThisOnTheRight(M); }
  template<typename D>
  void operator << (MatrixBase<D> & M,const GivensSequence& G) { G.applyThisOnTheLeft(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator >> (const GivensSequence& G, SubMatrix<MatrixType,PermutationType,IsSub> & M)
  { G.applyThisOnTheRight(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator << (SubMatrix<MatrixType,PermutationType,IsSub> & M,const GivensSequence& G)
  { G.applyThisOnTheLeft(M); }

  template<typename D>
  void operator >> (const GivensSequence::Transpose& G, MatrixBase<D> & M)
  { G.nested->applyTransposeOnTheRight(M); }
  template<typename D>
  void operator << (MatrixBase<D> & M,const GivensSequence::Transpose& G)
  { G.nested->applyTransposeOnTheLeft(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator >> (const GivensSequence::Transpose& G,
		    SubMatrix<MatrixType,PermutationType,IsSub> & M)
  { G.nested->applyTransposeOnTheRight(M); }
  template<typename MatrixType, int PermutationType, bool IsSub>
  void operator << (SubMatrix<MatrixType,PermutationType,IsSub> & M,
		    const GivensSequence::Transpose& G)
  { G.nested->applyTransposeOnTheLeft(M); }




  /* --- HEAVY CODE --------------------------------------------------------- */
  /* --- HEAVY CODE --------------------------------------------------------- */
  /* --- HEAVY CODE --------------------------------------------------------- */

  template<typename Derived,typename Rot>
  static void applyRotation( MatrixBase<Derived> & x,MatrixBase<Derived> & y, Rot j);


  /* --- Construction single ------------------------------------------------ */
  template<typename VectorBase>
  inline Givens::Givens(const VectorBase & v, int i, int j)
    :i(i), j(j)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    makeGivens(v, i, j);
  }
  template<typename VectorBase>
  inline Givens::Givens(VectorBase & v, int i, int j, bool apply)
    :i(i), j(j)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    makeGivens(v, i, j, apply);
  }

  template<typename VectorBase>
  inline void Givens::makeGivens(const VectorBase & v, int i, int j)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    makeGivens(v(i), v(j), i, j, NULL );
  }

  template<typename VectorBase>
  inline void Givens::makeGivens(VectorBase & v, int i, int j, bool apply)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    if( !apply ) makeGivens(v,i,j);
    else
      {
	makeGivens(v(i), v(j), i, j, &v(i));
	v(j)=0;
      }
  }

  /* --- Multiplication single ---------------------------------------------- */
  // M := M*G.
  template<typename Derived>
  void Givens::
  applyThisOnTheLeft(MatrixBase<Derived> & M) const
  {
    //original solution is not working because "innerstride" is not define in general
    // M.applyOnTheRight(i, j, G);
    typename Derived::ColXpr x(M.col(i));
    typename Derived::ColXpr y(M.col(j));
    applyRotation(x,y,G.adjoint());
  }

  // M := M*G.
  template<typename Derived>
  void Givens::
  applyThisOnTheLeftPartiel(MatrixBase<Derived> & M) const
  {
    if( (int(i)<M.cols())&&(int(j)<M.cols()) )
      M.applyOnTheRight(i, j, G);
    else if(int(i)<M.cols()) { assert( M.col(i).norm()<1e-6 ); }
    else if(int(j)<M.cols()) { assert( M.col(j).norm()<1e-6 ); }
  }

  // M := M*G'.
  template<typename Derived>
  void Givens::
  applyTransposeOnTheLeft(MatrixBase<Derived> & M) const
  {
    //original solution is not working because "innerstride" is not define in general
    // M.applyOnTheRight(i, j, G.adjoint());
    typename Derived::ColXpr x(M.col(i));
    typename Derived::ColXpr y(M.col(j));
    applyRotation(x,y,G);
  }

  // M := G*M.
  template<typename Derived>
  void Givens::
  applyThisOnTheRight(MatrixBase<Derived> & M) const
  {
    //original solution is not working because "innerstride" is not define in general
    // M.applyOnTheLeft(i, j, G);
    typename Derived::RowXpr x(M.row(i));
    typename Derived::RowXpr y(M.row(j));
    applyRotation(x,y,G);

  }


  // M := G'*M.
  template<typename Derived>
  void Givens::
  applyTransposeOnTheRight(MatrixBase<Derived> & M) const
  {
    //original solution is not working because "innerstride" is not define in general
    // M.applyOnTheLeft(i, j, G.adjoint());

    typename Derived::RowXpr x(M.row(i));
    typename Derived::RowXpr y(M.row(j));
    applyRotation(x,y,G.adjoint());
  }


  template<typename Derived,typename Rot>
  static void applyRotation( MatrixBase<Derived> & x,MatrixBase<Derived> & y, Rot j)
  {
    // TODO:
    //   - Depending on the value of has_direct_access<Derived>::ret, use the jacobi solution
    // instead of the iterative patch below.
    //   - correct the bug in internal::apply_rotation_in_the_plane.
    eigen_assert( x.size() == y.size() );
    typename Derived::Index size = x.size();
    double c=j.c(), s=j.s();
    for(typename Derived::Index a=0; a<size; ++a)
      {
	typename Derived::Scalar xa = x[a];
	typename Derived::Scalar ya = y[a];
	x[a] =  c * xa + s * ya;
	y[a] = -s * xa + c * ya;
      }
  }


  /* --- Multiplication single sub matrix ----------------------------------- */
  // M := M*G.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyThisOnTheLeft(SubMatrix<MatrixType,PermutationType,IsSub> & M) const
  {
    assert( 0<=i && int(i)<M.cols() && 0<=j && int(j)<M.cols() );
    typedef typename MatrixType::Index Index;
    for(Index r=0; r<M.rows(); ++r)
      {
	double & m_1 = M(r,i), & m_2 = M(r,j);
	const double x1=m_1, x2=m_2;
	// M*G == G'*M' -> apply the transpose of G.
	m_1 =  Gt.c() * x1 + internal::conj(Gt.s()) * x2;
	m_2 = -Gt.s() * x1 + internal::conj(Gt.c()) * x2;
      }
  }

  // M := M*G'.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyTransposeOnTheLeft(SubMatrix<MatrixType,PermutationType,IsSub> & M) const
  {
    assert( 0<=i && i<M.cols() && 0<=j && j<M.cols() );
    typedef typename MatrixType::Index Index;
    for(Index r=0; r<M.rows(); ++r)
      {
	double & m_1 = M(r,i), & m_2 = M(r,j);
	const double x1=m_1, x2=m_2;
	m_1 =  G.c() * x1 + internal::conj(G.s()) * x2;
	m_2 = -G.s() * x1 + internal::conj(G.c()) * x2;
      }
  }

  // M := G'*M.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyTransposeOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub>& M) const
  {
    assert( 0<=i && int(i)<M.rows() && 0<=j && int(j)<M.rows() );
    typedef typename MatrixType::Index Index;
    for(Index c=0; c<M.cols(); ++c)
      {
	double & m_1 = M(i,c), & m_2 = M(j,c);
	const double x1=m_1, x2=m_2;
	m_1 =  Gt.c() * x1 + internal::conj(Gt.s()) * x2;
	m_2 = -Gt.s() * x1 + internal::conj(Gt.c()) * x2;
      }
  }
  // M := G*M.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyThisOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub>& M) const
  {
    assert( 0<=i && i<M.rows() && 0<=j && j<M.rows() );
    typedef typename MatrixType::Index Index;
    for(Index c=0; c<M.cols(); ++c)
      {
	double & m_1 = M(i,c), & m_2 = M(j,c);
	const double x1=m_1, x2=m_2;
	m_1 =  G.c() * x1 + internal::conj(G.s()) * x2;
	m_2 = -G.s() * x1 + internal::conj(G.c()) * x2;
      }
  }

  // M := M*G.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyThisOnTheLeftPartiel(SubMatrix<MatrixType,PermutationType,IsSub>& M) const
  {
    if( (i<M.cols())&&(j<M.cols()) )
      applyThisOnTheLeft(M);
    else if(i<M.cols()) { assert( M.col(i).norm()<1e-6 ); }
    else if(j<M.cols()) { assert( M.col(j).norm()<1e-6 ); }
  }


  // M := G'*M.
  template<typename MatrixType, int PermutationType, bool IsSub>
  void Givens::
  applyTransposeOnTheRight(SubMatrix<MatrixType,PermutationType,IsSub>& M,
			   const int nbCols) const
  {
    assert( 0<=i && int(i)<M.rows() && 0<=j && int(j)<M.rows() );
    assert( 0<= nbCols && nbCols<=M.cols() );
    typedef typename MatrixType::Index Index;
    for(Index c=0; c<nbCols; ++c)
      {
	double & m_1 = M(i,c), & m_2 = M(j,c);
	const double x1=m_1, x2=m_2;
	m_1 =  Gt.c() * x1 + internal::conj(Gt.s()) * x2;
	m_2 = -Gt.s() * x1 + internal::conj(Gt.c()) * x2;
      }
  }



  /* --- Multiplication sequence -------------------------------------------- */
  // M := M*G.
  template<typename Derived>
  void GivensSequence::
  applyThisOnTheLeft(MatrixBase<Derived> & M) const
  {
    for (size_t i=0; i<G.size(); ++i)
      G[i].applyThisOnTheLeft(M);
  }

  // M := M*G.
  template<typename Derived>
  void GivensSequence::
  applyThisOnTheLeftReduced(MatrixBase<Derived> & M) const
  {
    for (size_t i=0; i<G.size(); ++i)
      G[i].applyThisOnTheLeftPartiel(M);
  }

  // M := M*G'.
  template<typename Derived>
  void GivensSequence::
  applyTransposeOnTheLeft(MatrixBase<Derived> & M) const
  {
    for (int i=G.size()-1; i>=0; --i)
      G[i].applyTransposeOnTheLeft(M);
  }

  // M := G*M.
  template<typename Derived>
  void GivensSequence::
  applyThisOnTheRight(MatrixBase<Derived> & M) const
  {
    for (int i=G.size()-1; i>=0; --i)
      { G[i].applyThisOnTheRight(M); }
  }

  // M := G'*M.
  template<typename Derived>
  void GivensSequence::
  applyTransposeOnTheRight(MatrixBase<Derived> & M) const
  {
    for (size_t i=0; i<G.size(); ++i)
      G[i].applyTransposeOnTheRight(M);
  }


  /* --- Multiplication display -------------------------------------------- */
  template<>
  inline MATLAB::
  MATLAB( unsigned int size,const GivensSequence & m1 )
  {
    if( size == 0 ) initMatrixColNull(0);
    else
      {
	MatrixXd Yex(size,size); Yex.setIdentity(); m1.applyThisOnTheLeft(Yex);
	initMatrix(Yex);
      }
  }


} // namespace soth


#endif //__SOTH_GIVENS__
