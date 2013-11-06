#ifndef __SOTH_BASEY__
#define __SOTH_BASEY__


#include "soth/api.hpp"
#include "soth/Algebra.hpp"
#include <Eigen/Householder>
#include "soth/Givens.hpp"

namespace soth
{

  class SOTH_EXPORT BaseY
  {
  protected:
    typedef MatrixXd::Index Index;
    typedef Diagonal<MatrixXd,0> HCoeffsType;
    typedef HouseholderSequence<MatrixXd,HCoeffsType> HouseholderSequenceType;
    typedef Diagonal<const MatrixXd,0> HCoeffsType_const;
    typedef HouseholderSequence<const MatrixXd,HCoeffsType_const> HouseholderSequenceType_const;

  protected:public:
    bool isExplicit;
    Index size;
    Index rank;
    MatrixXd matrixExplicit;
    MatrixXd householderEssential;

  public:
    // Empty construction with memory allocation.
    BaseY( const unsigned int & size );

    void computeExplicitly();
    void reset() { rank=0; isExplicit=false; }
  public:
    /* --- Accessor --- */
    MatrixXd& getHouseholderEssential() {return householderEssential;}
    const MatrixXd& getHouseholderEssential() const {return householderEssential;}

    Block<MatrixXd> getNextHouseholderEssential()
    {return householderEssential.bottomRightCorner(size-rank,size-rank);}


    void updateRank(Index r)      {      rank = r;    }
    void increaseRank(Index r)    {      rank += r;    }
    inline Index getRank(void) const { return rank; }

    HouseholderSequenceType_const getHouseholderSequence() const
    {
      HouseholderSequenceType_const res
	= HouseholderSequenceType_const(householderEssential,householderEssential.diagonal());
      //,trans=false,actualVectors=rank,shift=0
      res.setLength(rank);
      return res;
    }

    /* --- Multiplier --- */
    /* TODO: when not explicit, it is cheaper to work directly on the
     * result memory, while it is the opposit when explicit. What
     * should we do?
     */

    // v := Y*v = v*Y'.
    template< typename VectorGen >
    void applyThisOnVector( VectorGen & v ) const {applyThisOnTheRight(v);}
    // v := Y'*v = v*Y.
    template< typename VectorGen >
    void applyTransposeOnVector( VectorGen & v ) const {applyThisOnTheLeft(v);}

    // M := M*Y.
    template< typename Derived >
    void applyThisOnTheLeft( MatrixBase<Derived> & M ) const;
    // M := M*Y'.
    template< typename Derived >
    void applyTransposeOnTheLeft( MatrixBase<Derived> & M ) const;
    // M := Y*M.
    template< typename Derived >
    void applyThisOnTheRight( MatrixBase<Derived> & M ) const;
    // M := Y'*M.
    template< typename Derived >
    void applyTransposeOnTheRight( MatrixBase<Derived> & M ) const;

    BaseY& operator*= (const Givens& g);
    BaseY& operator*= (const GivensSequence& G);

    template< typename DerivedI, typename DerivedO >
    void multiply( const MatrixBase<DerivedI>& m, MatrixBase<DerivedO>& res ) const
    {
      if( isExplicit ) res = matrixExplicit*m;
      else { res=m; applyThisOnTheRight(res); }
    }
    template< typename DerivedI, typename DerivedO >
    void transposeMultiply( const MatrixBase<DerivedI>& m, MatrixBase<DerivedO>& res ) const
    {
      if( isExplicit ) res = matrixExplicit.transpose()*m;
      else { res=m; applyTransposeOnTheRight(res); }
    }

  };


  /* --- HEAVY CODE --------------------------------------------------------- */
  /* --- HEAVY CODE --------------------------------------------------------- */
  /* --- HEAVY CODE --------------------------------------------------------- */

 // // v := Y*v = v*Y'.
 // template< typename VectorGen >
 // void BaseY::
 // applyThisOnVector( VectorGen & v ) const
 // {
 //   if( isExplicit )
 //     {
	///*TODO*/ throw "TODO";
 //     }
 //   else
 //     {
	//matrixHH.applyThisOnVector(v);
 //     }
 // }

 // // v := Y'*v = v*Y.
 // template< typename VectorGen >
 // void BaseY::
 // applyTransposeOnVector( VectorGen & v ) const
 // {
 //   if( isExplicit )
 //     {
	///*TODO*/ throw "TODO";
 //     }
 //   else
 //     {
	//matrixHH.applyTransposeOnVector(v);
 //     }
 // }


  // M := M*Y.
  template< typename Derived >
  void BaseY::
  applyThisOnTheLeft( MatrixBase<Derived> & M ) const
  {
    if( isExplicit )
      {
	MatrixXd tmp = M;
	M = tmp*matrixExplicit;
      }
    else
      {	      M.applyOnTheRight(getHouseholderSequence());      }
  }

  // M := M*Y'.
  template< typename Derived >
  void BaseY::
  applyTransposeOnTheLeft( MatrixBase<Derived> & M ) const
  {
    if( isExplicit )
      {
	M = M*matrixExplicit.transpose();
      }
    else
      {	      M.applyOnTheRight(getHouseholderSequence().transpose());    }
  }

  // M := Y*M.
  template< typename Derived >
  void BaseY::
  applyThisOnTheRight( MatrixBase<Derived> & M ) const
  {
    if( isExplicit )
      {
	M = matrixExplicit*M;
      }
    else
      {
	      M.applyOnTheLeft(getHouseholderSequence());
      }
  }

  // M := Y'*M.
  template< typename Derived >
  void BaseY::
  applyTransposeOnTheRight( MatrixBase<Derived> & M ) const
  {
    if( isExplicit )
      {
	M = matrixExplicit.transpose()*M;
      }
    else
      {
	      M.applyOnTheLeft(getHouseholderSequence().transpose());
      }
  }


}




#endif //#ifndef __SOTH_BASEY__
