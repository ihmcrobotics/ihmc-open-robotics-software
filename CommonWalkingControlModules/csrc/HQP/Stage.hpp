#ifndef __SOTH_STAGE__
#define __SOTH_STAGE__

#include <Eigen/Core>
#include <list>
#include <string>

#include "soth/api.hpp"
#include "soth/BasicStage.hpp"
#include "soth/SubMatrix.hpp"
#include "soth/solvers.hpp"
#include "soth/Algebra.hpp"
#include "soth/Bound.hpp"
#include "soth/ActiveSet.hpp"
#include "soth/Givens.hpp"
#include "soth/Allocator.hpp"

namespace soth
{

  class BaseY;

  /* --- STAGE -------------------------------------------------------------- */
  /* --- STAGE -------------------------------------------------------------- */
  /* --- STAGE -------------------------------------------------------------- */

  class SOTH_EXPORT Stage
    :public BasicStage
  {
  public:
    typedef MatrixXd::Index Index;
    typedef SubMatrix<MatrixXd>::RowIndices Indirect;

  protected:

    using BasicStage::J;
    using BasicStage::bounds;
    using BasicStage::nc;
    using BasicStage::nr;
    using BasicStage::Y;

    /* Work range matrices. */
    MatrixXd W_;
    MatrixXd ML_;
    VectorXd e_;
    VectorXd lambda_;

    /* fullRankRows = Ir. defRankRows = In.
     * Ir = L.indirectRows() -- Irn = M.indirectRows(). In =D= Irn/Ir. */
    Indirect Ir, Irn, Iw, Im, Il;

    /* W = W_( :,[In Ir] ).
     * M = ML_( [In Ir],0:sizeM-1 ).
     * L = ML_( Ir,sizeM:sizeM+sizeL-1  ).
     * Lo = [0;L] = ML_( [In Ir],sizeM:sizeM+sizeL-1  ).
     *
     * W_*ML_ = W*[M [ zeros(In.size(),rank); L ] ]
     */
    SubMatrixXd M,L;
    SubMatrixXd W;
    SubMatrixXd Wr,Mr;
    SubVectorXd e,lambda;
    mutable VectorXd lambdadamped; // For debug only, can be removed on release.
    VectorXd lzfreezed;

    bool isWIdenty;
    /* sizeL = card(Ir). sizeM = previousRank. */
    unsigned int sizeM,sizeL;

    /* Memory allocators. */
    SubActiveSet<ActiveSet,Indirect> activeSet;
    AllocatorML freeML;

    /* Damping */
    MatrixXd Ld_,Ldwork_;
    mutable VectorXd edwork_;
    SubMatrixXd Ld,Ldwork;
    mutable SubVectorXd edwork;
    GivensSequence Wd;
    double dampingFactor;
    const static double DAMPING_FACTOR;

    /* Check if the stage has been reset, initialized, if the optimum
     * has been computed, and if the lagrange multipliers have been
     * computed. */
    bool isReset,isInit,isOptimumCpt,isLagrangeCpt,isDampCpt,isFreezed;

  public:
    Stage( const MatrixXd & J, const VectorBound & bounds, BaseY& Y  );
    Stage( const unsigned int nr, const unsigned int nc,
	   const double * Jdata, const Bound * bdata, const BaseY& Y );
    Stage( const unsigned int nr, const unsigned int nc,
	   const double * Jdata, const BaseY& Y );

    /* --- INIT ------------------------------------------------------------- */
    void setInitialActiveSet( void );
    void setInitialActiveSet( const cstref_vector_t & initialGuess,
			      bool checkTwin=false );
    cstref_vector_t getOptimalActiveSet( bool withTwin = false );

    void reset( void );
    /* Return the rank of the current COD = previousRank+size(L).
     * Give a non-const ref on Y so that it is possible to modify it.
     */
    void computeInitialCOD( BaseY & Yinit );

  protected:
    void nullifyLineDeficient( const Index row, const Index in_r );
    void computeInitialJY();
    void conditionalWinit( bool id );
    /* --- DOWN ------------------------------------------------------------- */
  public:
    /* Return true if the rank re-increase operated at the current stage. */
    bool downdate( const unsigned int position,
		   GivensSequence & Ydown );
    /* Return true if the rank decrease operated at the current stage. */
    bool propagateDowndate( GivensSequence & Ydown,
			    bool decreasePreviousRank );

  protected:
    void regularizeHessenberg( GivensSequence & Ydown );
    unsigned int nullifyACrossFromW( const  unsigned int position );
    void removeARowFromL( unsigned int row );
    void removeACrossFromW( const unsigned int & row, const unsigned int & col );

    /* --- UPD -------------------------------------------------------------- */
  public:
    /* Return the rank of the line where the rank re-decrease will occurs. */
    unsigned int update( const ConstraintRef & cst, GivensSequence & Yup );
    void propagateUpdate( GivensSequence & Ydown,
			  unsigned int decreasePreviousRank );
  protected:
    void addARow( const Index & mlrowup,bool deficient=false );

    /* --- SOLVE ------------------------------------------------------------ */
  public:
    /* Solve in the Y space. The solution has then to be multiply by Y: u = Y*Yu. */
    void computeSolution( VectorXd& Ytu ) const;

    void damp( void );
    template< typename VectorDerived >
    void applyDamping( MatrixBase<VectorDerived>& x  ) const;
    template< typename VD1,typename VD2 >
    void applyDamping( MatrixBase<VD1>& x,MatrixBase<VD2>& y  ) const;
    template< typename VectorDerived >
    void applyDampingTranspose( MatrixBase<VectorDerived>& x  ) const;
    template< typename VD1,typename VD2 >
    void applyDampingTranspose( MatrixBase<VD1>& x,const MatrixBase<VD2>& y  ) const;

    void damping( const double & factor ) { dampingFactor = factor; }
    double damping( void ) const { return dampingFactor; }
    bool useDamp( void ) const { return isDampCpt; }
    void dampBoundValue( const ConstraintRef & cst,const double & value );

    /* --- MULTIPLICATORS --------------------------------------------------- */
  public:
    /* The const functions simultaneously set up the lambda member. */
    template <typename D>
    void computeError(const VectorXd& Ytu, MatrixBase<D>& err ) const;
    void computeError(const VectorXd& Ytu );
    /* computeRho(.,.,false) is const. What trick can we use to explicit that? TODO. */
    void computeRho(const VectorXd& Ytu, VectorXd& Ytrho, bool inLambda = false );
    template <typename D>
    void computeLagrangeMultipliers( VectorXd& rho, MatrixBase<D>& l ) const;
    void computeLagrangeMultipliers( VectorXd& rho );

  protected:
    void computeErrorFromJu(const VectorXd& MLYtu);
    template <typename D>
    void computeErrorFromJu(const VectorXd& Ytu, MatrixBase<D>& err) const;
    void computeMLYtu( const VectorXd& Ytu,VectorXd& MLYtu ) const;

    /* --- ACTIVE SEARCH ---------------------------------------------------- */
  public:
    /* Return true if all bounds are checked with the specified tau.  If tau is
     * specified, the step is computed btw (with tau_out <= tau_in) and the
     * constraint to update is returned.
     */
    bool checkBound( const VectorXd& u0,const VectorXd& u1,
		     ConstraintRef*, double* tau );
    bool checkBound( const VectorXd& u0,const VectorXd& u1,
		     ConstraintRef& cstmax, double& taumax );
    bool maxLambda( const VectorXd& u, double & lmax, unsigned int & row ) const;
    void freezeSlacks(const bool & slacks = true);



    /* --- CHECK ------------------------------------------------------------ */
  public:
    /* WMLY = [ W*M W(:,1:rank)*L zeros(sizeA,nc-sizeM-sizeL) ]*Y' */
    void recompose( MatrixXd& WMLY ) const;
    void show( std::ostream& os, unsigned int stageRef, bool check=false ) const;
    void showActiveSet( std::ostream& os ) const;

    /* Return a sub matrix containing the active rows of J, in the
     * same order as given by W. J_ is a matrix where th full
     * J is stored (workspace). */
    SubMatrix<MatrixXd,RowPermutation>   Jactive( MatrixXd& J_ ) const ;
    MatrixXd  Jactive() const ;
    /* Return a sub vector containing the active rows of e, in the
     * same order as given by W. */
    SubVectorXd eactive( VectorXd& e_ ) const;
    VectorXd eactive() const ;

    bool testRecomposition( void ) const;
    bool testSolution( const VectorXd & solution ) const;
    bool testUnactiveTwins( void );

    /* For debug purpose, give the line of an active constraint (assert the activity). */
    Index where( unsigned int cst ) const;
    ConstraintRef which( unsigned int row ) const;
    bool isActive( unsigned int cst ) const;

  public:
    /* --- ACCESSORS --- */
    typedef TriangularView<SubMatrixXd,Lower> TriSubMatrixXd;
    typedef TriangularView<const_SubMatrixXd,Lower> const_TriSubMatrixXd;
    typedef VectorBlock<MatrixXd::RowXpr> RowL;
    typedef MatrixXd::RowXpr RowML;

    SubMatrixXd getM() { return M; }
    const_SubMatrixXd getM() const { return M; }
    SubMatrixXd getL() { return L; }
    const_SubMatrixXd getL() const { return L; }
    TriSubMatrixXd getLtri() { return L.triangularView<Lower>(); }
    const_TriSubMatrixXd getLtri() const { return L.triangularView<Lower>(); }
    TriSubMatrixXd getLdtri() { return Ld.triangularView<Lower>(); }
    const_TriSubMatrixXd getLdtri() const { return Ld.triangularView<Lower>(); }
    SubVectorXd gete() { return e; }
    const_SubVectorXd gete() const { return e; }
    SubVectorXd getLagrangeMultipliers() { return lambda; }
    const_SubVectorXd getLagrangeMultipliers() const { return lambda; }
    MatrixXd getWr() const { if(isWIdenty) return MatrixXd::Identity(sizeL,sizeL); else return Wr; }

    VectorXd getLagrangeDamped() const { return lambdadamped; }

    RowL rowL0( const Index r );
    RowML rowMrL0( const Index r );
    RowL rowML( const Index r );
    unsigned int rowSize( const Index r );

    /* TODO: sizeL and sizeM should be automatically determined from the corresponding indexes. */
    inline unsigned int nbConstraints( void ) const { return nr; }
    inline unsigned int sizeA( void ) const { return activeSet.nbActive(); }
    // sizeN = card(In) = sizeA-sizeL.
    inline int sizeN( void ) const { assert((int)sizeA()-sizeL>=0);return sizeA()-sizeL; }
    inline Index rank() const {return sizeL;}

    inline int getSizeM() const { return sizeM; }
    inline int getSizeL() const { return sizeL; }

    using BasicStage::getJrow;
    using BasicStage::getBoundRow;

  public:
    static double EPSILON;


  public: /* For debug purpose, could be remove on RELEASE. */
    std::string name;
  };

} // namespace soth


#endif // #ifndef __SOTH_STAGE__
