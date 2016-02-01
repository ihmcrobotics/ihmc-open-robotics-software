//#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "soth/debug.hpp"

#include "soth/Stage.hpp"
#include "soth/DestructiveColPivQR.hpp"
#include "soth/BaseY.hpp"
#include <Eigen/LU>

namespace soth
{



  using std::endl;

#define SOTH_STAGE_COMMON_CONSTRUCTOR                                                \
    W_(nr,nr),ML_(nr,nc),e_(nr),lambda_(nr)                                          \
                                                                                     \
    ,Ir(),Irn(),Iw(),Im(),Il()                                                       \
                                                                                     \
    ,M(ML_,&Irn,&Im),L(ML_,&Ir,&Il)                                                  \
    ,W(W_,&Iw,&Irn)                                                                  \
                                                                                     \
    ,Wr(W_,&Iw,&Ir),Mr(ML_,&Ir,&Im)                                                  \
    ,e(e_,&Iw),lambda(lambda_,&Iw)                                                   \
                                                                                     \
    ,sizeM(0),sizeL(0)                                                               \
                                                                                     \
    ,activeSet(nr,Iw)                                                                \
    ,freeML(nr)                                                                      \
                                                                                     \
    ,Ld_(nr,nr),Ldwork_(nr,nr),edwork_(nr)                                           \
    ,Ld(Ld_,false,false),Ldwork(Ldwork_,&Ld.getRowIndices(),&Ld.getColIndices())     \
    ,edwork(edwork_,&Ld.getRowIndices())                                             \
    ,Wd()                                                                            \
    ,dampingFactor( DAMPING_FACTOR )                                                 \
                                                                                     \
      ,isReset(false),isInit(false),isOptimumCpt(false),isLagrangeCpt(false),isDampCpt(false),isFreezed(false)

#define SOTH_STAGE_COMMON_INIT                                                       \
  do {                                                                               \
       Wd.reserve( int(nr*(nr+1)/2) );					             \
     } while(false)

  Stage::
  Stage( const MatrixXd & J, const VectorBound & bounds,BaseY & Y )
    :BasicStage(J,bounds,Y)
    ,SOTH_STAGE_COMMON_CONSTRUCTOR
  {
    SOTH_STAGE_COMMON_INIT;
  }

  Stage::
  Stage( const unsigned int IN_nr, const unsigned int IN_nc,
	 const double * IN_Jdata, const Bound * IN_bdata, const BaseY& IN_Y )
    :BasicStage(IN_nr,IN_nc,IN_Jdata,IN_bdata,IN_Y)
    ,SOTH_STAGE_COMMON_CONSTRUCTOR
  {
    SOTH_STAGE_COMMON_INIT;
  }

  Stage::
  Stage( const unsigned int IN_nr, const unsigned int IN_nc,
	 const double * IN_Jdata, const BaseY& IN_Y )
    :BasicStage(IN_nr,IN_nc,IN_Jdata,IN_Y)
    ,SOTH_STAGE_COMMON_CONSTRUCTOR
  {
    SOTH_STAGE_COMMON_INIT;
  }


  const double Stage::DAMPING_FACTOR = 1e-2;

  /* --- INITIALISATION OF THE COD ------------------------------------------ */
  /* --- INITIALISATION OF THE COD ------------------------------------------ */
  /* --- INITIALISATION OF THE COD ------------------------------------------ */

  void Stage::
  reset( void )
  {
    sotDEBUG(45) << "# In {" << name << endl;
    assert( !isReset );
    // TODO: disable the checks on release.
    isReset=true; isInit = false; isOptimumCpt = false;
    isLagrangeCpt = false; isDampCpt =false;
    isFreezed = false;

#ifndef NDEBUG
    ML_ = MatrixXd::Ones(nr,nc);
#endif

    sotDEBUG(45) << "# Out }" << name << endl;
  }

  void Stage::
  setInitialActiveSet( void )
  {
    activeSet.reset();

    /* TODO: set active set to TWIN only by default. */
    for( unsigned int i=0;i<nr;++i )
      {
	if( bounds[i].getType() != Bound::BOUND_TWIN ) continue;
	activeSet.activeRow( i,Bound::BOUND_TWIN );
      }
  }

  void Stage::
  setInitialActiveSet( const cstref_vector_t & initialGuess, bool checkTwin )
  {
    sotDEBUGIN(5);
    activeSet.reset();

    sotDEBUG(15) << name << " = " << initialGuess.size() << std::endl;
    for( unsigned int i=0;i<initialGuess.size();++i )
      {
	sotDEBUG(1) << "Activate " << initialGuess[i] << std::endl;
	activeSet.activeRow( initialGuess[i] );
      }

    if( checkTwin )
      {
	for( unsigned int i=0;i<nr;++i )
	  {
	    if( (bounds[i].getType()==Bound::BOUND_TWIN)
		&&(!activeSet.isActive(i)) )
	      activeSet.activeRow( i,Bound::BOUND_TWIN );
	  }
      }

    sotDEBUGOUT(5);
  }

  cstref_vector_t Stage::
  getOptimalActiveSet( bool withTwin )
  {
    cstref_vector_t res(sizeA());
    int loop=0;
    for( unsigned int i=0;i<sizeA();++i )
      {
	if( activeSet.whichBoundInv(i)!=Bound::BOUND_TWIN )
	  res[loop++]
	    = ConstraintRef(activeSet.whichConstraint(i),activeSet.whichBoundInv(i));
      }
    res.resize(loop);
    return res;
  }


  /* Compute ML=J(initIr,:)*Y. */
  void Stage::
  computeInitialJY( void )
  {
    sotDEBUG(5) << "b = " << (VectorBound)bounds << endl;
    if( sizeA()==0 )
      {
	sotDEBUG(5) << "Initial IR empty." << std::endl;
	assert(testUnactiveTwins());
	freeML.reset();
	ML_.setZero(); return;
      }

    activeSet.defrag(); // TODO: not always necessary? but cheap.

    VectorXi activeCst = activeSet;

    Block<MatrixXd> ML = ML_.topRows(sizeA());
    // DEBUG: const cast!! because SubMatrix does not support const&.
    ML = SubMatrix<MatrixXdRef,RowPermutation>( const_cast<MatrixXdRef&>(J),&activeCst );
    sotDEBUG(15) << "Ja = " << (MATLAB)ML << std::endl;

    freeML.resetTop(sizeA());

    const SubMatrix<VectorBoundRef,RowPermutation>
      ba( const_cast<VectorBoundRef&>(bounds),&activeCst );
    for( unsigned int r=0;r<sizeA();++r )
      {
	Bound::bound_t bt = activeSet.whichBound( activeCst[r] );
	e_[r]=ba[r].getBound( bt );
	if( bt == Bound::BOUND_INF )
	  {
	    ML.row(r) *= -1; e_[r] *= -1;
	  }
      }

    Y.applyThisOnTheLeft( ML );

    sotDEBUG(5) << "JY = " << (MATLAB)ML << std::endl;
    sotDEBUG(5) << "e = " << (MATLAB)e << std::endl;

    assert(testUnactiveTwins());
  }

  bool Stage::
  testUnactiveTwins( void )
  {
#ifndef NDEBUG
    sotDEBUG(5) << "Checking activation of twins." << std::endl;
    for( unsigned int cst=0;cst<nr;++cst )
      {
	if( (bounds[cst].getType() == Bound::BOUND_TWIN)
	    &&(! activeSet.isActive(cst)) )
	  return false;
      }
#endif
    return true;
  }

  /* The BaseY is given as a non const ref. It is equal to the Y ref
   * stored in the stage. */
  void Stage::
  computeInitialCOD( BaseY & Yinit )
  {
    /*
     * ML=J(initIr,:)*Y;
     * rank=Ir.size();  Ir=1:rank;
     * M=submatrix(ML,1:previousRank); L=submatrix(ML,previousRank+1:end);
     *
     * A=columnMajor(L)  % A==L'
     * qr(A);
     * RotationHouseHolder_list_t Yup( A );
     * Y=Y*Yup;
     *
     * for i=rank:-1:1
     *   if( L(i,i)!= 0 ) break;
     *     nullifyLineDeficient( i );
     */

    assert( isReset&&(!isInit ) );
    assert( &Y == & Yinit );
    isInit=true; isReset=false;
    sotDEBUG(5) << "J = " << (MATLAB)J << std::endl;

    /* Compute ML=J(initIr,:)*Y. */
    computeInitialJY();
    assert( Yinit.getRank() >= 0 && Yinit.getRank()<=(int)nc );
    const unsigned int previousRank = Y.getRank();

    isWIdenty = true;

    /* Set the size of M and L. L is supposed full rank yet. */
    /* M=submatrix(ML,1:previousRank); L=submatrix(ML,previousRank+1:end); */
    M.setColRange(0,previousRank);    M.setRowRange(0,sizeA());  sizeM=previousRank;
    L.setColRange(previousRank,nc);   L.setRowRange(0,sizeA());  sizeL=sizeA();

    sotDEBUG(15) << "MY = " << (MATLAB)M << std::endl;
    sotDEBUG(15) << "LY = " << (MATLAB)L << std::endl;
    sotDEBUG(5) << "e = " << (MATLAB)e << std::endl;
    sotDEBUG(25)<<"sizesAML = ["<<sizeA()<<", "<<sizeM<<", "<<sizeL<<"]."<<endl;

    if( (L.cols() == 0)||(L.rows()==0) )
    {
      sotDEBUG(5) << "col size of L is null, skip the end of initialization" << endl;
      L.setRowIndices(VectorXi());      L.setColIndices(VectorXi());
      sizeL=0;
      return;
    }

    /* 1. Right side rotation for partial triangularization:
     * J = I.[M L0 0 ].Y, with L0 non-full rank triangular. */

    /* A=L'; mQR=QR(A); */
    Block<MatrixXd> subY = Yinit.getNextHouseholderEssential();
    Block<MatrixXd> Lblock = ML_.block(0,previousRank,sizeA(),nc-previousRank);
    Transpose< Block<MatrixXd> > Lt = Lblock.transpose();
    Eigen::DestructiveColPivQR<Transpose< Block<MatrixXd> >,Block<MatrixXd> >
       mQR(Lt,subY, EPSILON);
    /*
    Transpose<SubMatrixXd> Lt = L.transpose();
    Eigen::DestructiveColPivQR<Transpose<SubMatrixXd>,Block<MatrixXd> >
      mQR(Lt,subY, EPSILON);
    */
    sotDEBUG(45) << "mR = " << (MATLAB) mQR.matrixR() << std::endl;
    sotDEBUG(45) << "mQ = " << (MATLAB)Yinit.getHouseholderEssential() << std::endl;
    sotDEBUG(47) << "ML_ = " << (MATLAB)ML_ << std::endl;

    /* L=triu(mQR'); */
    const VectorXi & P = mQR.colsPermutation().indices();
    L.setRowIndices(P);    M.setRowIndices(P);
    sotDEBUG(7) << "L0 = " << (MATLAB)L << std::endl;
    sotDEBUG(7) << "M0 = " << (MATLAB)M << std::endl;

    /* Artificial permutation to ensure that W == I at this point. */
    W.setRowIndices(P);
    sotDEBUG(5) << "e0 = " << (MATLAB)e << std::endl;

    /* 2. Left-side rotation to nullify the rank-def lines of L0: L0 == W.[0;L]. */
    /* for i=rank:-1:1
     *   if( L(i,i)!= 0 ) break;
     *     nullifyLineDeficient( i );
     * sizeL = mQR.rank();
     */
    assert( mQR.rank()>=0 && mQR.rank()<=int(sizeL) );
    const unsigned int rank = mQR.rank();

    conditionalWinit( sizeL==rank );
    while( sizeL>rank )
      {
    	/* Nullify the last line of L, which is of size rank. */
    	sotDEBUG(45) << "Nullify " << sizeL-1 << " / " << rank << std::endl;
    	nullifyLineDeficient( sizeL-1,rank );
     }
    L.setColRange(sizeM,sizeM+sizeL);
    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;
    sotDEBUG(5) << "W = " << MATLAB(W,isWIdenty) << std::endl;

    /* 3. Preparate for the iteration at the next stage: Y:=Y*Yup; */
    Yinit.increaseRank(sizeL);

    return;
  }

  void Stage::
  conditionalWinit( bool id )
  {
    if( id )
      { isWIdenty = true; }
    else if( isWIdenty )
      { isWIdenty = false; W.setIdentity(); }
    assert( id==isWIdenty );
  }

  /* Nullify the line <row> which is suppose to be length <in_r> (row-1 by
   * default) by given-rotations comming from the left.
   * Apply the same transformation on M, L and W. Remove the line from L
   * and reorder the lines of M and W by the way.
   * row is the position of the line, in_r its length.
   */
  void Stage::
  nullifyLineDeficient( const Index row, const Index in_r )
  {
    assert(! isWIdenty );
    /*
     * Jd = L.row(r);
     * foreach i in rank:-1:1
     *   if( Jd(i)==0 ) continue;
     *     gr= GR(L(Ir(i),i),Jd(i),i,r );
     *     L=gr*L;
     *     W=W*gr';
     * Ir >> r;
     * In << r;
     */
    const Index r = (in_r<0)?row-1:in_r;
    for( Index i=r-1;i>=0;--i )               // PSEUDOZEROS
      {
	Givens G1(L.col(i),i,row);

	G1.applyTransposeOnTheRight(Mr);
	G1.applyTransposeOnTheRight(L,r);
	G1.applyThisOnTheLeft(Wr);
      }
    removeARowFromL( row );

    sotDEBUG(15) << "W = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(15) << "L = " << (MATLAB)L << std::endl;
  }


  /* Remove a row of L, and commit the changes in M and W. */
  void Stage::
  removeARowFromL( unsigned int row )
  {
    assert(! isWIdenty );
    L.removeRow(row);
    M.pushRowFront(M.removeRow(row+sizeN()));
    sizeL--;
    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;
  }

  /* --- FREEZE ------------------------------------------------------------- */
  /* --- FREEZE ------------------------------------------------------------- */
  /* --- FREEZE ------------------------------------------------------------- */


  /* Freeze the equalities constraint whose lagrange multipliers are non zero.
   * In case of slack variables (when the arg is true), also modify the
   * objective function to an reachable value.
   */
  void Stage::
  freezeSlacks( const bool & slacks )
  {
    assert(isLagrangeCpt);
    sotDEBUG(55) << "# In { " << name << endl;

    EI_FOREACH( i,lambda )
      {
	if( std::abs(lambda[i])>EPSILON )
	  {
	    const unsigned int cstref = activeSet.mapInv(i);
	    if(! activeSet.isFreezed(cstref) )
	      {
		activeSet.freeze(cstref);
		if(!slacks)
		  { sotDEBUG(5)<<"Freeze cst "<<name<<":"<<cstref <<"."<<endl; }
	      }
	    /* Modify the bound when slack is l>0. */
	    if( slacks )
	      {
		e[i] += lambda[i];
		sotDEBUG(5)<<"Freeze cst "<<name<<":"<<cstref<<" to "<<e[i]<< endl;
	      }
	  }
      }

    if( isDampCpt&& sizeL>0 )
      {
	sotDEBUG(15) << "Freeze the damping." << name << endl;
	sotDEBUG(15) << "L = " << (MATLAB)L << endl;
	sotDEBUG(15) << "Ld = " << (MATLAB)Ld << endl;

	{
	  MatrixXd Wd1t = MatrixXd::Identity( sizeL,sizeL );
	  MatrixXd Wd2t = MatrixXd::Zero( sizeL,sizeL );
	  StackMatrix<MatrixXd,MatrixXd> Wd12t(Wd1t,Wd2t);
	  Wd.transpose() >> Wd12t;
	  L = Wd1t.lu().inverse() * Ld ;
	}

	// if( sizeM>0 )
	//   {
	//     MatrixXd Mx( sizeL,sizeM );
	//     StackMatrix<SubMatrixXd,MatrixXd> Mw(Mr,Mx);
	//     Wd >> Mw;
	//   }
	// VectorXd Wre;
	// if( isWIdenty )
	//   { applyDampingTranspose(e); }
	// else
	//   {
        //     VectorXd Wre = Wr.transpose()*e;
	//     VectorXd WetaWre = Wre;
	//     applyDampingTranspose(WetaWre);
	//     /* e = Wr*Wr'*e + W0*W0'*e
	//      * => W0*W0'e = e - Wr*Wr'*e
	//      * e := Wr*Weta*Wr'*e + W0*W0'*e = Wr*Weta*Wr'*e + e - Wr*Wr'*e
	//      */
	//     e += Wr*WetaWre - Wr*Wre;
	//   }
      }
    isFreezed =true;
    sotDEBUG(55) << "# Out } " << name << endl;
  }




  /* --- DOWNDATE ----------------------------------------------------------- */
  /* --- DOWNDATE ----------------------------------------------------------- */
  /* --- DOWNDATE ----------------------------------------------------------- */

  // Return true if the rank re-increase operated at the current stage.
  bool Stage::
  downdate( const unsigned int position,
	    GivensSequence & Ydown )
  {
    notifior(name,
	     ConstraintRef(activeSet.mapInv(position),
			   activeSet.whichBoundInv(position) ),
	     "downdate");
    /*
     *   gr = Neutralize row <position> in W <position>
     *   L=gr'*L
     *   bool res;
     *   if( L(In.last(),0) == 0
     *     // Rank deficience: regularize Hessenberg
     *	 Ydown = regularizeHessenberg
     *     res=false;
     *   else
     *     // No rank dec: quit
     *	 Ir << In.pop();
     *	 res=true;
     *   Ir >> r; In >> r;
     *   Unused << r;
     *   return res;
     */
    assert( isInit ); isLagrangeCpt=false; isOptimumCpt=false; isDampCpt=false;
    sotDEBUG(5) << " --- DOWNDATE ----------------------------" << std::endl;
    unsigned int colToRemove = nullifyACrossFromW( position );
    bool rankDef = int(colToRemove) >= sizeN();
    removeACrossFromW(position,colToRemove);

    if( rankDef )
      { // Full-rank line removed and no rank promotion: resorbe Hessenberg and propagate.
	// TODO: use the knowledge that the first colToRemove-sizeN()
	// of L are properly shaped and does not need any Hess.
	regularizeHessenberg(Ydown);
	L.removeCol(sizeL);

	sotDEBUG(5) << "W2 = " << MATLAB(W,isWIdenty) << std::endl;
	sotDEBUG(5) << "M2 = " << (MATLAB)M << std::endl;
	sotDEBUG(5) << "L2 = " << (MATLAB)L << std::endl;
	return false;
      }
    else
      {
	sotDEBUG(5) << "Nothing to do." << std::endl;
	return true;
      }
  }

  // Return true if the rank decrease operated at the current stage.
  bool Stage::propagateDowndate( GivensSequence & Ydown,
				 bool decreasePreviousRank )
  {
    assert( isInit ); isLagrangeCpt=false; isOptimumCpt=false; isDampCpt=false;
    /*
     * M=M*Ydown;
     * if(! decreasePreviousRank ) return true;
     * L.indices2().push_front( M.indice2().pop_back() );
     *
     * foreach i in In
     *   if( L(i,0) == 0 continue;
     *   Ir << i; In >> i;
     *   return true;
     *
     * Ydown += regularizeHessenberg
     * return false
     */

    /* Apply Ydown to ML:  MLi := MLi Ydown */
    EI_FOREACH( i,Irn )
      {
	RowL MLi = rowML(i);
	MLi << Ydown;
      }
    if( decreasePreviousRank ) return true;

    sotDEBUG(5) << "W0 = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(5) << "M0 = " << (MATLAB)M << std::endl;
    sotDEBUG(5) << "L0 = " << (MATLAB)L << std::endl;
    L.pushColFront( M.popColBack() );
    sizeM--;

    /* Check if one of the M's grown. */
    SubCol<MatrixXd> Ln( ML_,Irn,(int)sizeM );
    for( Index i=0;i<sizeN();++i )
      {
	if( std::abs(Ln(i)) > EPSILON )
	  {
	    /* Remove all the non-zero compononent of ML(i+1:end,sizeM). */
	    sotDEBUG(5) << "Found a non zero at "<<i << std::endl;
	    conditionalWinit(false);
	    Block<MatrixXd> ML(ML_,0,0,nr,sizeM+1);
	    for( Index j=i+1;j<sizeN();++j )  // PSEUDOZERO
	      {
		Givens G1( Ln,i,j );
		G1.transpose() >> M;
		G1.transpose() >> Ln;
		W << G1;
		assert( std::abs(ML_(Irn(j),sizeM))<EPSILON*EPSILON );
	      }

	    /* Commute the lines in L. */
	    M.permuteRows(i,sizeN()-1);
	    L.pushRowFront(Irn(sizeN()-1)); sizeL++;

	    sotDEBUG(5) << "M = " << (MATLAB)M << std::endl;
	    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;
	    sotDEBUG(5) << "W = " << MATLAB(W,isWIdenty) << std::endl;
	    sotDEBUG(5) << "sizeL = " << sizeL << std::endl;
	    return true;
	  }
      }

    /* No rank upgrade, resorbe hessenberg. */
    regularizeHessenberg(Ydown);
    L.popColBack();
    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;
    SubCol<MatrixXd>( ML_,Irn,(int)sizeM ).head( sizeN() ).setZero();  // PSEUDOZEROS

    return false;
  }


  /* Rotate W so that W is 1 on <row,col> (with col the smaller so that
   * W(row,col) is not null) and L|position is at worst hessenberg.
   */
  unsigned int Stage::
  nullifyACrossFromW( const  unsigned int row )
  {
    sotDEBUG(5) << "W0 = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(5) << "M0 = " << (MATLAB)M << std::endl;
    sotDEBUG(5) << "L0 = " << (MATLAB)L << std::endl;

    if( isWIdenty ) /* Nothing to do: there is already a 1 in W(row,row). */
      { return row; }

    /* Search for the first non-zero of W(row,:). */
    int col = 0;
    while( std::abs(W(row,col))< EPSILON ) col++;

    /* Nullify all the coefficients of W(row,col+1:end), until W(row,col)==1. */
    for( unsigned int i=col+1;i<sizeA();++i )
      {
	/* Compare ||rest||^2 = 1-(1-x)^2 ~ 2x, with x=Wrc. */
	if( std::abs(W(row,col)-1)< EPSILON*EPSILON/2 ) break;

	/* Wt(row,col) VS Wt(row,i) */
	Givens G1( W.row(row),col,i );
	W << G1;

	/* Apply the Given Rotation to ML. */
	const int rs = rowSize(i);
	if( rs>0 )
	  {
	    /* Apply on 2 specific lines of ML, so ML_ is OK. */
	    SubMatrix<MatrixXd,RowPermutation> ML( ML_,Irn );
	    G1.transpose() >> ML;
	    sotDEBUG(55) << "ML"<<i<<" = " << (MATLAB)ML <<endl;
	  }
      }

    sotDEBUG(15) << "W = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(25) << "M = " << (MATLAB)M << std::endl;
    sotDEBUG(25) << "L = " << (MATLAB)L << std::endl;
    sotDEBUG(5) << "colToRemove = " << col << endl;

    return col;
  }

  /* Remove a row, and commit the changes in M and W. */
  void Stage::
  removeACrossFromW( const unsigned int & row, const unsigned int & col  )
  {
    sotDEBUG(45) << "W = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(45) << "Wcidx = " << (MATLAB)W.getColIndices() << std::endl;
    sotDEBUG(45) << "Wridx = " << (MATLAB)W.getRowIndices() << std::endl;

    /* Store this range before modifying sizeA/sizeN. */
    const int rowRankInL = col-sizeN();

    activeSet.unactiveRow(row);
    const unsigned int wcoldown = W.removeCol(col);
    if( rowRankInL>=0 ) { L.removeRow(rowRankInL); sizeL--; }
    freeML.put(wcoldown);

    sotDEBUG(25) << "W = " << MATLAB(W,isWIdenty) << std::endl;
    sotDEBUG(25) << "M = " << (MATLAB)M << std::endl;
    sotDEBUG(25) << "L = " << (MATLAB)L << std::endl;
    sotDEBUG(50) << "Irn = " << (MATLAB)Irn << std::endl;
    sotDEBUG(45) << "Wcidx = " << (MATLAB)W.getColIndices() << std::endl;
    sotDEBUG(45) << "Wridx = " << (MATLAB)W.getRowIndices() << std::endl;
  }

  void Stage::regularizeHessenberg( GivensSequence & Ydown )
  {
    /*
     * for i=i0:rank-1
     *   gr = GR( L(Ir(i),i),L(Ir(i),i+1),i,i+1 );
     *   L = L*GR;
     *   Ydown.push_back( gr );
     */
    for( unsigned int i=0;i<sizeL;++i )
      {
	RowML MLi = rowMrL0(i);
	Givens G1(MLi,sizeM+i,sizeM+i+1,true);

	for( unsigned r=i+1;r<sizeL;++r )
	  {
	    RowML MLr = rowMrL0(r) ;
	    MLr << G1;
	  }
	Ydown.push(G1);
      }
  }


  /* --- UPDATE ------------------------------------------------------------- */
  /* --- UPDATE ------------------------------------------------------------- */
  /* --- UPDATE ------------------------------------------------------------- */

  unsigned int Stage::
  update( const ConstraintRef & cst,GivensSequence & Yup )
  {
    //usleep(1000*0.01);
    sotDEBUG(5) << " --- UPDATE ----------------------------" << std::endl;
    notifior(name,cst,"update");
    assert( isInit ); isLagrangeCpt=false; isOptimumCpt=false; isDampCpt=false;
    assert( (cst.type==Bound::BOUND_SUP)||(cst.type==Bound::BOUND_INF) );
    /*
     * Inew = Unused.pop();
     * Row JupY = row(Inew);
     * JupU = Jup*Y;
     * double norm2=0; double rankJ=0;
     * for i=n:-1:1
     *   norm2+=JupY(i)^2;
     *   if norm2!=0 rankJ=i; break;
     *
     * Ir << Inew
     * W(Inew,Inew)=1;
     * if rankJ>sizeM+rank
     *   // Rank increase
     *   for i=rankJ:-1:sizeM+rank+1
     *     Gr gr; gr.init( JupY,i,i-1,0 ); prod(JupY,gr);
     *     Yup.push_back( gr );
     *     return false;
     * else
     *   // No rank increase;
     *   nullifyLineDeficient(Inew);
     *   return true;
     */
    const Index wrowup = activeSet.activeRow( cst );
    sotDEBUG(45) << "Wcidx = " << (MATLAB)W.getColIndices() << endl;
    sotDEBUG(45) << "Wridx = " << (MATLAB)W.getRowIndices() << endl;

    /* Add a coefficient to e. */
    double sign = cst.sign();
    e[wrowup] = sign*bounds[cst.row].getBound(cst.type);
    sotDEBUG(5) << "cst=" << cst << "  (" << sign << ")." << endl;

    /* Choose the first available row in ML. */
    const unsigned int wcolup = freeML.get();
    assert( (wcolup >= 0)&&(wcolup<nr) );
    sotDEBUG(5) << " wc=" << wcolup << endl;

    /* Add a line to ML. */
    RowML JupY = ML_.row(wcolup);
    JupY = sign*J.row(cst.row); Y.applyThisOnTheLeft(JupY);
    sotDEBUG(5) << "JupY = " << (MATLAB)JupY << endl;
    sotDEBUG(5) << "JupY = " << JupY << endl;

    /* Determine the rank on the new line. */
    double norm2=0; unsigned int rankJ=sizeM;
    for( Index i=nc-1;i>=int(sizeM);--i )
      {
	norm2=JupY(i)*JupY(i); // TODO: should be +=
	if( norm2>EPSILON*EPSILON )
	  {
	    sotDEBUG(45) << "Oversize value is x"<<i<<" = " << JupY(i)<<std::endl;
	    rankJ=i+1;
	    break;
	  }
      }
    sotDEBUG(5) << "rankUp = " << rankJ << endl;

    if( rankJ>sizeM+sizeL )
      { /* Rank increase: remove the tail of JuY. */
	for( Index i=rankJ-1;i>int(sizeM+sizeL);--i )
	  {
	    Givens G1(JupY,i-1,i,true);
	    Yup.push(G1);
	  }
	addARow(wcolup);
	L.pushColBack(sizeM+sizeL-1);
      }
    else
      { /* No rank increase: regularize. */
	conditionalWinit(false);
	if( rankJ>sizeM )
	  {
	    addARow(wcolup);
	    nullifyLineDeficient(sizeL-1,rankJ-sizeM);
	  }
	else
	  { /* L-part of the new line null, no need to regularize. */
	    addARow(wcolup,true);
	  }
      }

    sotDEBUG(5) << "W = " << MATLAB(W,isWIdenty) << endl;
    sotDEBUG(5) << "M = " << (MATLAB)M << endl;
    sotDEBUG(5) << "L = " << (MATLAB)L << endl;
    return rankJ;
  }


  void Stage::
  propagateUpdate( GivensSequence & Ydown,
		   unsigned int decreasePreviousRank )
  {
    assert( isInit ); isLagrangeCpt=false; isOptimumCpt=false; isDampCpt=false;
    /*
     * M=M*Ydown;
     * if(! decreasePreviousRank ) return true;
     * L.indices2().push_front( M.indice2().pop_back() );
     *
     * foreach i in In
     *   if( L(i,0) == 0 continue;
     *   Ir << i; In >> i;
     *   return true;
     *
     * Ydown += regularizeHessenberg
     * return false
     */

    bool defDone = decreasePreviousRank<=sizeM;
    if(! defDone )
      {
	if( sizeL>0 ) M.pushColBack( L.popColFront() );
	else M.pushColBack(sizeM);
	sizeM++;
      }
    sotDEBUG(5) << "M = " << (MATLAB)M << endl;
    sotDEBUG(5) << "L = " << (MATLAB)L << endl;

#ifdef SOTH_DEBUG
    {
      MatrixXd Yex(nc,nc); Yex.setIdentity(); Ydown.applyThisOnTheLeft(Yex);
      sotDEBUG(5) << (MATLAB)Yex << endl;
    }
#endif

    for( unsigned int i=0;i<sizeA();++i )
      {
	RowL MLi = rowML(i);
	Ydown.applyThisOnTheLeftReduced(MLi);
      }
    sotDEBUG(5) << "M = " << (MATLAB)M << endl;
    sotDEBUG(5) << "L = " << (MATLAB)L << endl;

    /* sizeM already increased, so sM+sL is the last col of the Hessenberg. */
    if( sizeM+sizeL<=decreasePreviousRank )
      { /* L increased a column. */
	if( sizeL>0 )L.pushColBack( sizeM+sizeL-1 );
      }
    else if(! defDone )
      { /* Rank decrease ongoing... */
	conditionalWinit(false);
	const int rdef = decreasePreviousRank-sizeM;
	assert(rdef>=0 && rdef<int(sizeL) );
	nullifyLineDeficient( rdef,rdef );
      }
    else
      { /* Already lost the rank, nothing to do. */
      }

    sotDEBUG(5) << "M = " << (MATLAB)M << endl;
    sotDEBUG(5) << "L = " << (MATLAB)L << endl;
  }

  void Stage::
  addARow( const Index & wcolup,bool deficient )
  {
    sotDEBUG(5) << "W0 = " << MATLAB(W,isWIdenty) << endl;

    if(! isWIdenty )
      {
	/* clean W. */
	// const Index last = W.cols()-1;
	// W.row(last).setZero();
	// W.col(last).setZero();
	// W(last,last) = 1.0;
	const SubMatrixXd & const_W = W;
	const Index & wrowup = const_W.getRowIndices(sizeA()-1);
	W_.row( wrowup ) .setZero();
	W_.col( wcolup ) .setZero();
	W_(wrowup,wcolup ) = 1.0;
      }

    /* Increase M, L and W. */
    M.pushRowBack( wcolup );
    if(!deficient)
      {
	L.pushRowBack( wcolup );	sizeL++;
      }

    sotDEBUG(5) << "W = " << MATLAB(W,isWIdenty) << endl;
    sotDEBUG(5) << "M = " << (MATLAB)M << endl;
    sotDEBUG(5) << "L = " << (MATLAB)L << endl;
  }

  /* --- SOLVER ------------------------------------------------------------- */
  /* --- SOLVER ------------------------------------------------------------- */
  /* --- SOLVER ------------------------------------------------------------- */

  /* --- DIRECT ------------------------------------------------------------- */
  /* Zu=Linv*(Ui'*ei-Mi*Yu(1:rai_1,1)); */
  void Stage::
  computeSolution( VectorXd& Ytu ) const
  {
    assert( isInit );
    if (sizeL==0)
    {
      sotDEBUG(10) << "size of L is 0, skipping solve" << std::endl;
      return;
    }
    sotDEBUG(5) << "e = " << (MATLAB)e << std::endl;
    sotDEBUG(25) << "Ytu = " << (MATLAB)Ytu << std::endl;

    const VectorBlock<VectorXd> zm = Ytu.head( sizeM );
    VectorBlock<VectorXd> We = Ytu.segment( sizeM,sizeL );

    if(! isWIdenty ) { sotDEBUG(45) << "Wr = " << (MATLAB)Wr << endl; }
    else             { sotDEBUG(45) << "W = " << MATLAB(W,isWIdenty) << endl; }

    if( isWIdenty ) We = e;
    else            We = Wr.transpose()*e;
    sotDEBUG(25) << "Wre = " << (MATLAB)We << std::endl;

    if( sizeM>0 )   We -= Mr*zm;
    sotDEBUG(5) << "Wre_Mru = " << (MATLAB)We << std::endl;

    if( isDampCpt )
      {
        applyDampingTranspose(We);
        sotDEBUG(5) << "Ld = " << (MATLAB)Ld << std::endl;
        sotDEBUG(5) << "Wde = " << (MATLAB)We << std::endl;
        soth::solveInPlaceWithLowerTriangular(Ld,We);
      }
    else
      {
        soth::solveInPlaceWithLowerTriangular(L,We);
      }
    sotDEBUG(5) << "LiWe = " << (MATLAB)We << std::endl;
    sotDEBUG(45) << "Ytu = " << (MATLAB)Ytu << std::endl;
  }


  /* Compute the damped triangle Ld so that [L;damp.I] = Wd[L0;0],
   * and store the transformation.
   */
  void Stage::
  damp( void  )
  {
    //assert(! isDampCpt );
    if (sizeL==0)
      {
	sotDEBUG(10) << "size of L is 0, skipping damp" << std::endl;
	Ld.setColRange(0,0);
	Ld.setRowRange(0,0);
	Wd.clear();
	isDampCpt=true; return;
      }
    if( isFreezed )
      {
	sotDEBUG(10) << "Freezed, no more damping." << std::endl;
	Wd.clear();
	Ld.setColRange(0,sizeL);
	Ld.setRowRange(0,sizeL);
	Ld = L;
	isDampCpt=true; return;
      }

    const double damp = dampingFactor;
    Ld.setColRange(0,sizeL);
    Ld.setRowRange(0,sizeL);
    Wd.clear();

    Ld = L;
    Ldwork = MatrixXd::Identity(sizeL,sizeL)*damp;

    StackMatrix<SubMatrixXd,SubMatrixXd> Lw(Ld,Ldwork);

    sotDEBUG(45) << "Ld0 = " << (MATLAB)Lw << endl;

    for( unsigned int r=0;r<sizeL;++r )
      {
    	for( int c=r;c>=0;--c )
    	  {
    	    Givens G1( Lw.col(c),c,sizeL+r );
    	    G1.transpose() >> Lw;
	    Wd.push(G1);
    	  }
      }

    sotDEBUG(55) << "Ld1 = " << (MATLAB)Lw << endl;
    sotDEBUG(5) << "Ld = " << (MATLAB)Ld << endl;
    sotDEBUG(5) << "Wd = " << MATLAB(sizeL*2,Wd) << endl;
    isDampCpt=true;
    return;
  }

  /* Compute x := Wd x, with Wd the rotation so that
   * [L;I] = Wd [L0;0].
   */
  template< typename VectorDerived >
  void Stage::
  applyDamping( MatrixBase<VectorDerived>& x  ) const
  {
    typedef MatrixBase<VectorDerived> VectorBase;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    assert( isDampCpt );
    assert( x.size() == L.cols() && x.size() == L.rows() );
    if (sizeL==0)
      {
	sotDEBUG(10) << "size of L is 0, skipping damp" << std::endl;
	return;
      }

    edwork.setZero();
    StackMatrix<VectorDerived,SubVectorXd> ew(x,edwork);
    sotDEBUG(45) << "ed0 = " << (MATLAB)ew << endl;
    Wd >> ew;

    sotDEBUG(55) << "ed1 = " << (MATLAB)ew << endl;
    sotDEBUG(5) << "ed = " << (MATLAB)x << endl;
    return;
  }

  template< typename VD1,typename VD2 >
  void Stage::
  applyDamping( MatrixBase<VD1>& x,MatrixBase<VD2>& y  ) const
  {
    typedef MatrixBase<VD1> VectorBase1;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase1);
    typedef MatrixBase<VD1> VectorBase2;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase2);
    assert( isDampCpt );
    assert( x.size() == L.cols() && x.size() == L.rows() );
    if (sizeL==0)
      {
	sotDEBUG(10) << "size of L is 0, skipping damp" << std::endl;
	return;
      }

    y.setZero();
    StackMatrix<VD1,VD2> ew(x,y);
    sotDEBUG(45) << "ed0 = " << (MATLAB)ew << endl;
    sotDEBUG(45) << "W = " << MATLAB(sizeL*2,Wd) << endl;

    Wd >> ew;

    sotDEBUG(45) << "ed1 = " << (MATLAB)ew << endl;
    sotDEBUG(5) << "ed = " << (MATLAB)x << endl;
    return;
  }

  /* Compute x := Wd'x, with Wd the rotation so that
   * [L;I] = Wd [L0;0].
   */
  template< typename VectorDerived >
  void Stage::
  applyDampingTranspose( MatrixBase<VectorDerived>& x  ) const
  {
    typedef MatrixBase<VectorDerived> VectorBase;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase);
    assert( isDampCpt );
    assert( x.size() == L.cols() && x.size() == L.rows() );
    if (sizeL==0)
      {
	sotDEBUG(10) << "size of L is 0, skipping damp" << std::endl;
	return;
      }

    edwork.setZero();
    StackMatrix<VectorDerived,SubVectorXd> ew(x,edwork);
    sotDEBUG(45) << "ed0 = " << (MATLAB)ew << endl;
    sotDEBUG(45) << "Wd = " << MATLAB(sizeL*2,Wd) << endl;
    Wd.transpose() >> ew;

    sotDEBUG(45) << "ed1 = " << (MATLAB)ew << endl;
    sotDEBUG(5) << "ed = " << (MATLAB)x << endl;
    return;
  }

  template< typename VD1,typename VD2 >
  void Stage::
  applyDampingTranspose( MatrixBase<VD1>& x,const MatrixBase<VD2>& y  ) const
  {
    typedef MatrixBase<VD1> VectorBase1;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase1);
    typedef MatrixBase<VD1> VectorBase2;
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorBase2);
    assert( isDampCpt );
    assert( x.size() == L.cols() && x.size() == L.rows() );
    assert( y.size() == L.cols() && y.size() == L.rows() );
    if (sizeL==0)
      {
	sotDEBUG(10) << "size of L is 0, skipping damp" << std::endl;
	return;
      }

    edwork=y*dampingFactor;
    StackMatrix<VD1,SubVectorXd> ew(x,edwork);
    sotDEBUG(45) << "ed0 = " << (MATLAB)ew << endl;
    Wd.transpose() >> ew;

    sotDEBUG(55) << "ed1 = " << (MATLAB)ew << endl;
    sotDEBUG(5) << "ed = " << (MATLAB)x << endl;
    return;
  }

  void Stage::
  dampBoundValue( const ConstraintRef & cst,const double & value )
  {
    activeSet.dampBoundValue( cst,value );
  }



  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* --- TODO DEPRECATED ---------------------------------------------------- */

  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* err = Ju-e = W [M L 0] Y^u - e
   * where MLYtu has already been computed.
   */
  template< typename D>
  void Stage::
  computeErrorFromJu(const VectorXd& MLYtu,MatrixBase<D>& err) const
  {
    assert( false && "DEPRECATED" );

    assert( MLYtu.size() == int(sizeA()) );
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D>);

    if( isWIdenty ) err.noalias() = MLYtu-e;
    else            err.noalias() =  W*MLYtu-e;
  }

  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* Compute W'Ju = MLYtu = M*Ytu.head + L*Ytu.tail.
   * TODO: After a little bit of reformulation, Just need to compute Mn.z.
   */
  void Stage::
  computeMLYtu( const VectorXd& Ytu,VectorXd& MLYtu ) const
  {
    assert( false && "DEPRECATED" );

    assert(Ytu.size() == int(nc));
    MLYtu.noalias() = M*Ytu.head(sizeM);
    MLYtu.tail(sizeL).noalias()
      += getLtri()*Ytu.segment(sizeM, sizeL);

    sotDEBUG(45) << "MLYtu = " << (MATLAB)MLYtu << endl;
  }

  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* err = Ju-e = W [M L 0] Y^u - e
   */
  template <typename D>
  void Stage::
  computeError(const VectorXd& Ytu, MatrixBase<D>& err) const
  {
    assert( false && "DEPRECATED" );

    assert(Ytu.size() == int(nc) );
    assert( isInit );
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatrixBase<D>);

    // TODO: temporary allocation in this expression. Manage it?
    VectorXd MLYtu; computeMLYtu(Ytu,MLYtu);
    computeErrorFromJu(MLYtu,err);
  }

  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* Compute the error from scratch, and stored it in lambda. */
  void Stage::
  computeError(const VectorXd& Ytu)
  {
    assert( false && "DEPRECATED" );

    assert( isInit );
    computeError(Ytu,lambda);
    isLagrangeCpt =true;
  }

  /* --- TODO DEPRECATED ---------------------------------------------------- */
  /* Compute the error from already compute MLYtu, and stored it in lambda. */
  void Stage::
  computeErrorFromJu(const VectorXd& MLYtu)
  {
    assert( false && "DEPRECATED" );

    computeErrorFromJu(MLYtu,lambda);
    sotDEBUG(1) << "l = " << (MATLAB)lambda << endl;
    isLagrangeCpt =true;
  }

  /* --- INDIRECT ----------------------------------------------------------- */


  /* Compute J' (Ju-e) in Y base:
   * Ytrho = -Y'J'(Ju-e) = [ M L ]' ( W'e - [M L] Ytu ).
   * If damped, J~[J;damp*I] => Ytrho = Ytrho - damp^2*Ytu.
   */
  void Stage::
  computeRho(const VectorXd& Ytu, VectorXd& Ytrho, bool inLambda )
  {
    assert( isInit );
    assert(Ytu.size() == int(nc) );

    if( inLambda )
      { /* Lagrang of the last stage. */
	if( sizeN()==0 )
	  {
	    sotDEBUG(5) << "sizeN==0, rho=0." << std::endl;
	    lambda.setZero();
	  }
	else if( isWIdenty )
	  {
	    /* Compute lambda = e-W*MLYtu by the way. */
	    sotDEBUG(5) << "L is void, [ M L 0 ] = [ M 0 ] ." << std::endl;
	    assert( sizeL==0 );
	    VectorXd MnYtu = M*Ytu.head(sizeM) - e;
	    lambda.noalias() = MnYtu;
	  }
	else
	  {
	    /* Compute lambda = e-W*MLYtu by the way. */
	    sotDEBUG(5) << "Accounting for W." << std::endl;
	    assert(! isWIdenty );
	    //TODO : manage temporary memory ?
	    Block<SubMatrixXd> Mn = M.topRows(sizeN());
	    SubMatrixXd::ColsBlockXpr Wn = W.leftCols(sizeN());
	    VectorXd MnYtu = Mn*Ytu.head(sizeM) - Wn.transpose()*e;
	    lambda.noalias() = Wn*MnYtu;
	  }
	isLagrangeCpt =true;
      }

    if( sizeM==0&&sizeL==0 ) { /* Nothing to do */ }
    else if( sizeM==0 )
      {
	assert( sizeL>0 );
 	VectorBlock<const VectorXd> z = Ytu.head(sizeL);

	VectorXd MLz = VectorXd::Zero( sizeA() );
	MLz.tail( sizeL ) = L*z;
	if( isWIdenty ) MLz -= e;
	else            MLz -= W.transpose()*e;

	Ytrho.head(sizeL).noalias() = -L.transpose()*MLz.tail(sizeL);
     }
    else if( sizeL==0 )
      {
	VectorBlock<const VectorXd> zbar = Ytu.head(sizeM);
	VectorXd MLz; MLz = M*zbar;
	if( isWIdenty ) MLz -= e;
	else            MLz -= W.transpose()*e;
	Ytrho.head(sizeM)         .noalias() = -M.transpose()*MLz;
      }
    else
      {
	assert( sizeM>0 && sizeL>0 );
	VectorBlock<const VectorXd> zbar = Ytu.head(sizeM), z = Ytu.segment(sizeM,sizeL);

	VectorXd MLz; MLz = M*zbar;
	if( isWIdenty ) MLz -= e;
	else            MLz -= W.transpose()*e;
	MLz.tail(sizeL) += L*z;

	Ytrho.head(sizeM)         .noalias() = -M.transpose()*MLz;
	Ytrho.segment(sizeM,sizeL).noalias() = -L.transpose()*MLz.tail(sizeL);
      }
    sotDEBUG(5) << "rho = " << Ytrho << endl;

    if( isDampCpt )
      {
	const double & eta = dampingFactor;
	VectorBlock<const VectorXd> zbar = Ytu.head(sizeM), z = Ytu.segment(sizeM,sizeL);

     	if( sizeM>0 )
	  {
	    for( int i=0;i<(int)sizeM;++i )
	      {
		if(! M.col(i).isZero(EPSILON) )
		  Ytrho[i] -= eta*eta*zbar[i];
	      }
	  }
     	if( sizeL>0 ) Ytrho.segment(sizeM,sizeL).noalias() -= eta*eta*z;

     	sotDEBUG(5) << "rhod = " << Ytrho << endl;
      }
  }


   /** input: rho_under_i = {ro_1, ..., ro_i}
    * on return:
    * lambda_i =  Wr_i*L_i^{-T}*rho_i
    * rho_under_{i-1} = rho_under_{i-1} + Mr_i^T*L_i^{-T}*rho_i
//???    * rho_i = L_i^{-T}*rho_i (should not be useful).
    */
  template <typename D>
  void Stage::
  computeLagrangeMultipliers( VectorXd& rho, MatrixBase<D>& l ) const
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY( MatrixBase<D> );
    assert( isInit );
    assert( rho.rows() == int(nc) );
    assert( l.size() == int(sizeA()) );

    if( sizeL==0 )
      {	l.setZero(); return; }
    VectorBlock<VectorXd> rho_i = rho.segment(sizeM,sizeL);
    VectorBlock<VectorXd> rho_under = rho.head(sizeM);
    sotDEBUG(5) << "rho = " << (MATLAB)rho_i << endl;

    // if( isDampCpt )
    //   {
    //  	solveInPlaceWithUpperTriangular(Ld.transpose(), rho_i);
    // 	applyDampingTranspose( rho_i );
    //   }
    // else
      {
	sotDEBUG(5) << "L = " << (MATLAB)L << endl;
	solveInPlaceWithUpperTriangular(L.transpose(), rho_i);
      }

    if( isWIdenty ) l.noalias() = rho_i;
    else            l.noalias() = Wr * rho_i;

    if( sizeM>0 )
      {	rho_under.noalias() -= Mr.transpose()*rho_i;      }

    sotDEBUG(5) << "Lirho = " << (MATLAB)rho_i << endl;
  }

  void Stage::
  computeLagrangeMultipliers( VectorXd& rho )
  {
    assert( isInit );
    computeLagrangeMultipliers(rho,lambda);
    sotDEBUG(1) << "l = " << (MATLAB)lambda << endl;
    isLagrangeCpt=true;
  }

  /* --- BOUND -------------------------------------------------------------- */

  /* Return true if the previous tau is correct (ie maxlocal(tau)>taumax */
  bool Stage::checkBound( const VectorXd& u0,const VectorXd& u1,
                          ConstraintRef& cstmax, double& taumax )
  {
    bool res = true;
    for( unsigned int i=0;i<nr;++i )
      {
        if( activeSet.isActive(i) ) continue;
        assert( bounds[i].getType()!=Bound::BOUND_TWIN );

        /* This has already been computed and could be avoided... TODO. */
        double val0 = J.row(i)*u0;
        double val1 = J.row(i)*u1;
        const Bound & b = bounds[i];
        sotDEBUG(5) << "bound = " << b << endl;
        sotDEBUG(5) <<"Ju0="<<val0<<"  --  Ju1="<<val1<<" -- dval="<<val1-val0<<endl;

        Bound::bound_t btype1;
        if( isDampCpt ) btype1=b.check(val1,activeSet.getBoundDamping(i),EPSILON);
        else            btype1=b.check(val1,EPSILON);
        sotDEBUG(5) << "Damping " << activeSet.getBoundDamping(i).first << ","
                    << activeSet.getBoundDamping(i).second << std::endl;
        if( btype1!=Bound::BOUND_NONE )
          {
            assert( (btype1==Bound::BOUND_INF)||(btype1==Bound::BOUND_SUP) );
            sotDEBUG(5) << "Violation at " <<name <<" "
                        << ((btype1==Bound::BOUND_INF)?"-":"+")<<i << std::endl;

            //Bound::bound_t btype0 = b.check(val0,EPSILON);
            Bound::bound_t btype0 = b.check(val0);
            if( btype0==Bound::BOUND_NONE )
              {
                // assert( ( b.checkSaturation(val0,EPSILON)!=btype1 )
                //         && "Was saturated, and is now violate." );

                const double & bval = b.getBound(btype1);
                double btau = (bval-val0)/(val1-val0);
                assert(btau>=0); assert(btau<1);
		sotDEBUG(25) << "tau="<<btau<<endl;
                if( btau<taumax )
                  {
                    sotDEBUG(1) << "Max violation (tau="<<btau<<") at "<<name <<" "
                                << ((btype1==Bound::BOUND_INF)?"-":"+")<<i << std::endl;
                    res=false;
                    taumax=btau; cstmax = ConstraintRef(i,btype1);
                  }
              }
            else
             {
                if(taumax==1)
                  {
                    sotDEBUG(5) << "Violation Ju0 and Ju1 at " <<name <<" "
                                << ((btype1==Bound::BOUND_INF)?"-":"+")<<i << std::endl;
                    taumax=1-EPSILON; cstmax = ConstraintRef(i,btype1);
                    res=false;
                  }
              }
          }
      }
    return res;
  }


 bool Stage::checkBound( const VectorXd& u0,const VectorXd& u1,
                          ConstraintRef* cstptr, double* tauptr )
  {
    if( (tauptr==NULL)&&(cstptr==NULL) )
      { double tau=1; ConstraintRef cst; return checkBound(u0,u1,cst,tau); }
    else if( (tauptr!=NULL)&&(cstptr==NULL) )
      { ConstraintRef cst; return checkBound(u0,u1,cst,*tauptr); }
    else if( (tauptr==NULL)&&(cstptr!=NULL) )
      { double tau=1; return checkBound(u0,u1,*cstptr,tau); }
    else /* ie when ( (tauptr!=NULL)&&(cstptr!=NULL) ). */
      { return checkBound(u0,u1,*cstptr,*tauptr); }
  }


  bool Stage:: // TODO: Ytu could be passed instead of u. TODO! u is not usefull any more.
  maxLambda( const VectorXd& /*u*/, double & lmax,unsigned int& row ) const
  {
    /* TODO: unactive the search for TWINS. */

    bool res=false;
    EI_FOREACH( i,lambda )
      {
	const unsigned int cstref = activeSet.mapInv(i);
	Bound::bound_t btype = activeSet.whichBound(cstref);

	if( activeSet.isFreezed(cstref) ) continue;
	switch( btype ) // TODO: the code is the same for +/-, factorize or change the sign of J.
	  {
	  case Bound::BOUND_TWIN:
	    break; // Nothing to do.
	  case Bound::BOUND_SUP:
	    sotDEBUG(5) << name<<": row"<<i<<", cst"<<which(i) << ": l=" << lambda(i,0) << endl;
	    if( -lambda[i]>lmax )
	      {
		// double Ju = J.row(cstref)*u;
		// if( Ju<=bounds[cstref].getBound( Bound::BOUND_SUP )+EPSILON )
		  {
		    res=true;
		    lmax=-lambda[i];
		    row=i;
		  }
	      }
	    break;
	  case Bound::BOUND_INF:
	    sotDEBUG(5) << name<<": row"<<i<<", cst"<<which(i) << ": l=" << lambda(i,0) << endl;
	    if( -lambda[i]>lmax ) // TODO: change the sign of the bound-inf cst.
	      {
		// double Ju = J.row(cstref)*u;
		// if( bounds[cstref].getBound( Bound::BOUND_INF )-EPSILON<=Ju )
		  {
		    res=true;
		    lmax=-lambda[i];
		    row=i;
		  }
	      }
	    break;
	  case Bound::BOUND_NONE:
	  case Bound::BOUND_DOUBLE:
	    assert( (btype!=Bound::BOUND_NONE)&&(btype!=Bound::BOUND_DOUBLE) );
	    break;
	  }
      }
    return res;
  }

  /* --- ACCESSORS ---------------------------------------------------------- */
  /* --- ACCESSORS ---------------------------------------------------------- */
  /* --- ACCESSORS ---------------------------------------------------------- */

  /* Get line <r> of the matrix [ L 0 .. 0 ]. */
  Stage::RowL Stage::rowL0( const Index r )
  {
    return ML_.row(Ir(r)).tail(nc-sizeM);
  }

  /* Get line <r> of the matrix [ Mr L 0 .. 0 ] (- Mr = M(Ir,:) -)*/
  Stage::RowML Stage::rowMrL0( const Index r )
  {
    return ML_.row(Ir(r));
  }

  /* Get line <r> of the matrix [ M [0;L] 0 ], headed to the non zero part. */
  Stage::RowL Stage::rowML( const Index r )
  {
    return ML_.row(Irn(r)).head(rowSize(r));
  }

  unsigned int Stage::rowSize( const Index r )
  {
    if( r<sizeN() ) return sizeM;
    else return std::min( (unsigned int)(sizeM+r-sizeN()+1),nc );
 }

  /* --- TEST RECOMPOSE ----------------------------------------------------- */
  /* --- TEST RECOMPOSE ----------------------------------------------------- */
  /* --- TEST RECOMPOSE ----------------------------------------------------- */

  /* WMLY = [ W*M W(:,1:rank)*L zeros(sizeA,nc-sizeM-sizeL) ]*Y' */
  void Stage::
  recompose( MatrixXd& WMLY ) const
  {
    sotDEBUGPRIOR(+40);
    if( sizeA()==0 )
      {
	assert( (sizeL==0)&&(M.rows()==0)&&(L.rows()==0)
		&&(L.cols()==0)&&(W.rows()==0)&&(W.rows()==0) );
	WMLY.resize(0,nc);
	return;
      }
    WMLY.resize(sizeA(),nc); WMLY.setZero();
    if( isWIdenty ) WMLY.block(0,0,sizeA(),sizeM) = M;
    else            WMLY.block(0,0,sizeA(),sizeM) = W*M;

    if(! isWIdenty )
      { sotDEBUG(25) << "Wr = " << (MATLAB)Wr << endl; }
    sotDEBUG(25) << "L = " << (MATLAB)L << std::endl;
    if (sizeL != 0)
    {
      if( isWIdenty ) WMLY.block(0,sizeM,sizeA(),sizeL) = L;
      else            WMLY.block(0,sizeM,sizeA(),sizeL) = Wr*L;
    }
    sotDEBUG(25) << "WML = " << (MATLAB)WMLY << std::endl;

    Y.applyTransposeOnTheLeft(WMLY);
    sotDEBUG(25) << "WMLY = " << (MATLAB)WMLY << std::endl;
 }

  /* Return true iff Jactive=recompose and eactive=e. */
  bool Stage::
  testRecomposition( void ) const
  {
    /* When damp is used, recomposition is wrong after freezing. */
    if( isFreezed ) return true;
    MatrixXd Jrec; recompose(Jrec);
    MatrixXd Ja_;   SubMatrix<MatrixXd,RowPermutation> Ja = Jactive(Ja_);
    sotDEBUG(15) << "Jrec="<<(MATLAB)Jrec << endl;
    sotDEBUG(15) << "Ja="<<(MATLAB)Ja << endl;
    bool res; double norm=0;
    if( sizeA() ) { norm=(Jrec-Ja).norm(); res = (norm<=10*EPSILON); }
    else res = ( (Jrec.cols()==Ja.cols())&&(Jrec.rows()==Jrec.rows()) );
    sotDEBUG(5) <<"% J: Recomposition  " << ((res)?"OK":"wrong")
		<< " (||.||="<<norm<<")."<< std::endl;

    VectorXd ea_;
    bool vres=true; // DEBUG: the test is wrong when the stage has been freezed.
    // if( sizeA() ) vres = (e-eactive(ea_)).norm()<=EPSILON;
    // else vres = (e.size()==eactive(ea_).size() );

    // sotDEBUG(15) << "e="<<(MATLAB)e << endl;
    // sotDEBUG(15) << "ea="<<(MATLAB)eactive(ea_) << endl;
    // sotDEBUG(5) <<"% e: Recomposition  " << ((vres)?"OK.":"wrong.") << std::endl;

    return res&&vres;
  }

  /* Check that J*u = Wr*Wr'*e. */
  bool Stage::
  testSolution( const VectorXd & /*solution*/ ) const
  {
    /* TODO: test solution is broken. Fix it! */
    
    // VectorXd Ju = Jactive()*solution;

    // // SubMatrixXd Wr(W_,W.getRowIndices(),L.getRowIndices());
    // VectorXd Pwre = Wr*Wr.transpose()*eactive();
    // sotDEBUG(5) << "de = " << (MATLAB)(Ju-Pwre) << endl;

    // return (Ju-Pwre).norm() < EPSILON;
    return true;
  }


  Stage::Index Stage::
  where( unsigned int cst ) const
  { return activeSet.map(cst); }
  ConstraintRef Stage::
  which( unsigned int row ) const
  {
    assert( row<sizeA() );
    ConstraintRef res;
    res.row = activeSet.mapInv(row);
    res.type = activeSet.whichBound( res.row );
    return res;
  }
  bool Stage::
  isActive( unsigned int cst ) const
  {
    assert( cst<nr );
    return activeSet.isActive(cst);
  }

   /* Return a sub matrix containing the active rows of J, in the
   * same order as given by W. J_ is a matrix where th full
   * J is stored (workspace). */
  SubMatrix<MatrixXd,RowPermutation> Stage::
  Jactive( MatrixXd& J_ ) const
  {
    J_.resize(nr,nc); J_.setConstant(-1.11111);
    for( unsigned int cst=0;cst<nr;++cst )
      {
	if( activeSet.isActive(cst) )
	  {
	    const unsigned int row = activeSet.map(cst);
	    J_.row( Iw(row) ) = activeSet.sign(cst)*J.row(cst);
	  }
      }

    return SubMatrix<MatrixXd,RowPermutation> (J_,Iw);
  }
  MatrixXd Stage::Jactive() const {  MatrixXd J_; return Jactive(J_);  }

  /* Return a sub vector containing the active rows of e, in the
   * same order as given by W. */
  SubVectorXd Stage::
  eactive( VectorXd& e_ ) const
  {
    e_.resize(nr); e_.setConstant(-1.11111);

    for( unsigned int cst=0;cst<nr;++cst )
      {
	if( activeSet.isActive(cst) )
	  {
	    e_(activeSet.where(cst))
	      = activeSet.sign(cst)
	      * bounds[cst].getBound(activeSet.whichBound(cst));
	  }
      }

     return SubVectorXd(e_,Iw);
  }
  VectorXd Stage::eactive() const
  {
    VectorXd e_;
    return eactive(e_);
  }

  void Stage::
  show( std::ostream& os, unsigned int stageRef, bool check ) const
  {
    sotDEBUGPRIOR(+20);

    if( sotDEBUG_ENABLE(55) ) activeSet.disp( sotDEBUGFLOW,false );
    os << "sa{"<<stageRef<<"} = " << (MATLAB)sizeA() << endl;
    os << "r{"<<stageRef<<"} = " << (MATLAB)sizeL << endl;
    os << "sn{"<<stageRef<<"} = " << (MATLAB)sizeN() << endl;
    os << "sm{"<<stageRef<<"} = " << (MATLAB)sizeM << endl;

    sotDEBUG(45) << "Iw{"<<stageRef<<"} = " << (MATLAB)Iw << std::endl;
    sotDEBUG(45) << "Irn{"<<stageRef<<"} = " << (MATLAB)Irn << std::endl;
    sotDEBUG(25) << "ML_{"<<stageRef<<"} = " << (MATLAB)ML_ << std::endl;
    sotDEBUG(5) << "erec{"<<stageRef<<"} = " << (MATLAB)eactive() << std::endl;

    os << "a{"<<stageRef<<"} = " << activeSet << std::endl;
    os << "J{"<<stageRef<<"} = " << (MATLAB)Jactive() << std::endl;
    os << "e{"<<stageRef<<"} = " << (MATLAB)e << std::endl;
    os << "W{"<<stageRef<<"} = " << MATLAB(W,isWIdenty) << std::endl;
    os << "M{"<<stageRef<<"} = " << (MATLAB)M << std::endl;
    os << "L{"<<stageRef<<"} = " << (MATLAB)L << std::endl;
    if( isDampCpt )
      {
	os << "Ld{"<<stageRef<<"} = " << (MATLAB)Ld << std::endl;
	os << "Wd{"<<stageRef<<"} = " << MATLAB(sizeL*2,Wd) << std::endl;
      }


    if( check )
    {
      MatrixXd Jrec; recompose(Jrec);
      if (Jrec.rows()>0)
      {
        sotDEBUG(5) << "Jrec="<<(MATLAB)Jrec << endl;
        if((Jrec-Jactive()).norm()>1e-6)
	  os << "Jrec{"<<stageRef<<"} = " << (MATLAB)Jrec << std::endl;
        else os <<"% Recomposition OK. " << std::endl;
	if((e-eactive()).norm()<=1e-6)
	  sotDEBUG(5) <<"% Recomposition e OK. " << std::endl;
	else os << "% Recomposition e not OK. " << std::endl;
      }
      else
      {
        sotDEBUG(5) << "Jrec="<<(MATLAB)Jrec << endl;
        os <<"% Recomposition OK. " << std::endl;
        sotDEBUG(5) <<"% Recomposition e OK. " << std::endl;
      }
    }
    if( isLagrangeCpt )
      {
	os << "lag{"<<stageRef<<"} = " << (MATLAB)lambda << std::endl;
      }
  }

  void Stage::
  showActiveSet( std::ostream& os ) const
  {
    os << activeSet << std::endl;
  }


  //double Stage::EPSILON = 1e-8; // TODO: should be 1e-8
  double Stage::EPSILON = 1e-8;

} // namespace soth
