#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 15
#include "soth/debug.hpp"
//#include "soth/COD.hpp"  // DEBUG
#include "soth/HCOD.hpp"
#include <boost/foreach.hpp>
#include <sys/time.h>
#include <fstream>

namespace soth
{

  HCOD::
  HCOD( unsigned int inSizeProblem, unsigned int nbStage )
  :
    sizeProblem(inSizeProblem)
    ,Y(sizeProblem)
    ,stages(0)
    ,solution(sizeProblem)
    ,uNext(sizeProblem),Ytu(sizeProblem),YtuNext(sizeProblem),rho(sizeProblem)
    ,freezedStages(0)
    ,isReset(false),isInit(false),isSolutionCpt(false),withDamp(false)
  {
# ifndef NDEBUG
    //sotDebugTrace::openFile();
#endif

    stages.reserve(nbStage);
  }

  /* --- SETTERS/GETTERS ---------------------------------------------------- */
  /* --- SETTERS/GETTERS ---------------------------------------------------- */
  /* --- SETTERS/GETTERS ---------------------------------------------------- */

  void HCOD::
  pushBackStage( const MatrixXd & J, const VectorBound & bounds )
  {
    stages.push_back( stage_ptr_t(new soth::Stage( J,bounds,Y )));
    isInit=false;
  }
  void HCOD::
  pushBackStage( const unsigned int & nr, const double * Jdata, const Bound * bdata )
  {
    stages.push_back(stage_ptr_t(new soth::Stage( nr,sizeProblem,Jdata,bdata,Y )));
    isInit=false;
  }
  void HCOD::
  pushBackStage( const unsigned int & nr, const double * Jdata )
  {
    stages.push_back( stage_ptr_t(new soth::Stage( nr,sizeProblem,Jdata,Y )) );

    isInit=false;
  }

  void HCOD::
  pushBackStages( const std::vector<MatrixXd> & J,
		  const std::vector<VectorBound> & bounds )
  {
    assert( J.size() == bounds.size() );
    for( unsigned int i=0;i<J.size();++i )
      {
	pushBackStage( J[i],bounds[i] );
      }
  }

  Stage& HCOD::
  stage( unsigned int i )
  {
    assert( i<stages.size() );
    return *stages[i];
  }
  const Stage& HCOD::
  stage( unsigned int i ) const
  {
    assert( i<stages.size() );
    return *stages[i];
  }

  void HCOD::
  setInitialActiveSet()
  {
    for( stage_iter_t iter=stages.begin();iter!=stages.end();++iter )
      {
	(*iter)->setInitialActiveSet();
      }
  }

  void HCOD::
  setInitialActiveSet( const cstref_vector_t& Ir0,unsigned int k )
  {
    sotDEBUG(5) << "Ir["<<k<<"]"<<std::endl;
    stage(k).setInitialActiveSet(Ir0,true);
  }

  cstref_vector_t HCOD::
  getOptimalActiveSet( unsigned int k )
  {
    return stage(k).getOptimalActiveSet();
  }

  std::vector<cstref_vector_t> HCOD::
  getOptimalActiveSet()
  {
    std::vector<cstref_vector_t> res(stages.size());
    for( int k=0;k<(int)stages.size();k++ )
      res[k] = getOptimalActiveSet(k);
    return res;
  }

  void HCOD::
  setInitialActiveSet( const  std::vector<cstref_vector_t> & Ir0)
  {
    sotDEBUGIN(5);
    assert(Ir0.size() == stages.size() );

    for( int k=0;k<(int)stages.size();k++ )
      setInitialActiveSet(Ir0[k],k);
    sotDEBUGOUT(5);
  }

  int HCOD::sizeA() const
  {
    int s=0;
    for (size_t i=0; i<stages.size(); ++i)
      s+= stages[i]->sizeA();
    return s;
  }

  int HCOD::rank() const
  {
    int r=0;
    for (size_t i=0; i<stages.size(); ++i)
      r+= stages[i]->rank();
    return r;
  }

  void HCOD::setNameByOrder( const std::string root )
  {
    for (size_t i=0; i<stages.size(); ++i)
      {
	std::ostringstream os; os<<root<<i;
	stages[i]->name = os.str();
      }
  }
  void HCOD::
  notifiorRegistration( const Stage::listener_function_t & f, int stageRank )
  {
#ifndef WITHOUT_NOTIFIOR
    if( stageRank ==-1 )
      for (size_t i=0; i<stages.size(); ++i)
	{
	  stages[i]->notifior.connect(f);
	}
    else  stages[stageRank]->notifior.connect(f);
#endif
  }

  void HCOD::setDamping( const double & d )
  {
    useDamp( d!=0.0 );
    for( stage_iter_t iter = stages.begin();iter!=stages.end();++iter )
      {	(*iter)->damping(d);      }
  }
  double HCOD::getMaxDamping() const
  {
    if(! useDamp() ) return 0;
    double maxD=-1;
    for( stage_citer_t iter = stages.begin();iter!=stages.end();++iter )
      {
	double d = (*iter)->damping();
	assert( d>=0 );
	if( d>maxD ) maxD=d;
      }
    return maxD;
  }


  /* --- DECOMPOSITION ------------------------------------------------------- */
  /* --- DECOMPOSITION ------------------------------------------------------- */
  /* --- DECOMPOSITION ------------------------------------------------------- */

  void HCOD::
  reset( void )
  {
    isReset=true;
    isInit=false;
    isSolutionCpt=false;

    Y.reset();
    solution.setZero(); Ytu.setZero();
    for( stage_iter_t iter = stages.begin();iter!=stages.end();++iter )
      {   (*iter)->reset();   }
  }

  void HCOD::
  initialize( void )
  {
    if(! isReset) reset(); // TODO: should it be automatically reset?
    assert( isReset&&(!isInit) );

    /* Compute the initial COD for each stage. */
    for( unsigned int i=0;i<stages.size();++i )
      {
	assert( stages[i]!=0 );
	sotDEBUG(5) <<" --- STAGE " <<i
		    << " ---------------------------------" << std::endl;
	stages[i]->computeInitialCOD(Y);
      }
    isReset=false; isInit=true;
  }
  void HCOD::
  update( const unsigned int & stageUp,const ConstraintRef & cst )
  {
    assert(isInit);
    GivensSequence Yup;
    unsigned int rankDef = stages[stageUp]->update(cst,Yup);
    for( unsigned int i=stageUp+1;i<stages.size();++i )
      {
	stages[i]->propagateUpdate(Yup,rankDef);
      }
    updateY(Yup);
  }
  void HCOD::
  update( stage_iter_t stageIter,const ConstraintRef & cst )
  {
    assert(isInit);
    sotDEBUG(5) << "Update " << (*stageIter)->name <<", "
		<< cst << std::endl;
    GivensSequence Yup;
    unsigned int rankDef = (*stageIter)->update(cst,Yup);
    for( ++stageIter;stageIter!=stages.end();++stageIter )
      {
	(*stageIter)->propagateUpdate(Yup,rankDef);
      }
    updateY(Yup);
  }
  void HCOD::
  downdate( const unsigned int & stageDown, const unsigned int & rowDown )
  {
    assert(isInit);
    GivensSequence Ydown;
    bool propag=stages[stageDown]->downdate(rowDown,Ydown);
    for( unsigned int i=stageDown+1;i<stages.size();++i )
      {
     	propag = stages[i]->propagateDowndate(Ydown,propag);
      }
    updateY(Ydown);
  }
  void HCOD::
  downdate( stage_iter_t stageIter,const unsigned int & rowDown )
  {
    assert(isInit);

    if( useDamp() )
      {
	Stage & s = **stageIter;
	ConstraintRef cst = s.which(rowDown);
	double Ju = s.getJrow(cst.row).transpose()*solution;
	const Bound& b = s.getBoundRow(cst.row);
	double b_Ju = b.getBound( cst.type ) - Ju;
	s.dampBoundValue( cst,b_Ju );
	sotDEBUG(5) << "Damp cst " << cst << " with " << b_Ju << std::endl;
      }

    sotDEBUG(5) << "Downdate " << (*stageIter)->name<<", row "<<rowDown
		<<" (cst="<<(*stageIter)->which(rowDown)<<")."<< std::endl;
    GivensSequence Ydown;
    bool propag=(*stageIter)->downdate(rowDown,Ydown);
    for( ++stageIter;stageIter!=stages.end();++stageIter )
      {
     	propag = (*stageIter)->propagateDowndate(Ydown,propag);
      }
    updateY(Ydown);
  }



  void HCOD::
  updateY( const GivensSequence& Yup )
  {
    Y *= Yup;
    Yup.applyTransposeOnTheRight(Ytu);
    Yup.applyTransposeOnTheRight(YtuNext); /* TODO: this second multiplication could
					    * be avoided. */
  }


  /* --- COMPUTE ------------------------------------------------------------ */
  /* --- COMPUTE ------------------------------------------------------------ */
  /* --- COMPUTE ------------------------------------------------------------ */

  /* Assume the variable 'Ytu' contains Y^T*u.
   * The result is stored in the stages (getLagrangeMultipliers()).
   */
  void HCOD::
  computeLagrangeMultipliers( const unsigned int & stageRef )
  {
    assert( isSolutionCpt );
    assert( stageRef<=stages.size() );

    stage_iter_t iter=stages.begin()+stageRef;

    if( iter==stages.end() )
      {   rho = - Ytu;     }
    else
      {
	(*iter)->computeRho(Ytu,rho,true); // This is Ytrho, not rho.
      }
    sotDEBUG(5) << "rho = " << (MATLAB)rho << std::endl;

    while ( iter!=stages.begin() )
      {
	iter--;
	(*iter)->computeLagrangeMultipliers(rho);
      }
  }

  void HCOD::
  computeSolution(  bool compute_u )
  {
    assert(isInit);

    YtuNext.setZero();
    for( unsigned int i=0;i<stages.size();++i )
      {
        stages[i]->computeSolution(YtuNext);
      }

    if( compute_u )
      {
        Y.multiply(YtuNext,uNext);
        sotDEBUG(5) << "u0 = " << (MATLAB)solution << std::endl;
        sotDEBUG(5) << "u1 = " << (MATLAB)uNext << std::endl;
      }

    sotDEBUG(5) << "Ytu0 = " << (MATLAB)Ytu << std::endl;
    sotDEBUG(5) << "Ytu1 = " << (MATLAB)YtuNext << std::endl;
    isSolutionCpt=true;
  }
  void HCOD::
  damp( void )
  {
    assert(isInit);
    if(! useDamp() )  return;
    sotDEBUG(1) << "Damp ... " << std::endl;
    BOOST_FOREACH( stage_ptr_t sptr,stages )
      {	sptr->damp();      }
  }

  double HCOD::
  computeStepAndUpdate( void )
  {
    assert(isSolutionCpt);

    double tau = 1.0; ConstraintRef cst;
    stage_iter_t stageUp;
    for( stage_iter_t iter = stages.begin(); iter!=stages.end(); ++iter )
      {
	sotDEBUG(5) << "Check stage " << (*iter)->name << "." << std::endl;
	if(! (*iter)->checkBound( solution,uNext,cst,tau ) )
	  stageUp=iter;
      }
    if( tau<1 )
      {
	update(stageUp,cst);
     }
    return tau;
  }
  double HCOD::
  computeStep( void )
  {
    assert(isSolutionCpt);

    double tau = 1.0; ConstraintRef cst;
    stage_iter_t stageUp;
    for( stage_iter_t iter = stages.begin(); iter!=stages.end(); ++iter )
      {
	sotDEBUG(5) << "Check stage " << (*iter)->name << "." << std::endl;
	if(! (*iter)->checkBound( solution,uNext,cst,tau ) )
	  stageUp=iter;
      }
    return tau;
  }

  void HCOD::
  makeStep( double tau, bool compute_u )
  {
    Ytu *= (1-tau); Ytu += tau*YtuNext;
    if( compute_u ) { solution *= (1-tau); solution += tau*uNext; }
    sotDEBUG(5) << "Ytu = " << (MATLAB)Ytu << std::endl;
    sotDEBUG(5) << "u = " << (MATLAB)solution << std::endl;
  }
  void HCOD::
  makeStep( bool compute_u )
  {
    Ytu = YtuNext;
    if( compute_u ) { solution = uNext; }
    sotDEBUG(5) << "Ytu = " << (MATLAB)Ytu << std::endl;
    sotDEBUG(5) << "u = " << (MATLAB)solution << std::endl;
  }

  /* Return true iff the search is positive, ie if downdate was
   * needed and performed. */
  bool HCOD::
  searchAndDowndate( const unsigned int & stageRef )
  {
    assert( stageRef<=stages.size() );
    double lambdamax = Stage::EPSILON; unsigned int row;

    const stage_iter_t refend = stages.begin()+ std::min((unsigned)(stages.size()),stageRef+1);
    stage_iter_t stageDown =  refend;
    for( stage_iter_t iter = stages.begin(); iter!=refend; ++iter )
      {
	if( (*iter)->maxLambda( solution,lambdamax,row ) )
	  stageDown=iter;
      }
    if( refend!=stageDown )
      {
	downdate(stageDown,row);
	return true;
      }
    return false;
  }

  bool HCOD::
  search( const unsigned int & stageRef )
  {
    assert( stageRef<=stages.size() );
    double lambdamax = Stage::EPSILON; unsigned int row;
    const stage_iter_t refend = stages.begin()+std::min((unsigned)(stages.size()),stageRef+1);
    stage_iter_t stageDown = refend;
    for( stage_iter_t iter = stages.begin(); iter!=refend; ++iter )
      {
	if( (*iter)->maxLambda( solution,lambdamax,row ) )
	  stageDown=iter;
      }
    return ( refend!=stageDown );
  }


  /* --- ACTIVE SEARCH ------------------------------------------------------ */
  /* --- ACTIVE SEARCH ------------------------------------------------------ */
  /* --- ACTIVE SEARCH ------------------------------------------------------ */
  void HCOD::
  debugOnce(std::string filename,bool keepOpen)
  {
    if (filename.length()==0)
      sotDebugTrace::openFile();
    else
      {
	std::cout <<filename<<std::endl;
	soth::sotDebugTrace::openFile(filename.c_str());
      }

    isDebugOnce = ! keepOpen; 
    sotDEBUG(15) << "Test trace"<<std::endl;
  }


    /* TODO:
       - Finish updateY

       - Compact the final active set (init aset is suppose to **TODO**
       be row-compact). / Assert this hypothesis.
       - Make all the necessary functions public, and externalize the algo. **TODO**

       - computation of the violation per stages **DONE**
       - computation of tau **DONE**
       - translation of cst_ref in triple<stage_ref,cst_ref,bound_ref> **NOT NECESSARY**
       - compute min lambda,w **DONE**
       - compute du from Ytdu and Ytu from u.  **STUPID**

       - Test with empty stages, full stages, rank 0 stages. **DONE**
       - Build a test with fixed-values matrices of all ranks. **DONE**

    */

  void HCOD::activeSearch( VectorXd & u )
  {
    // if( isDebugOnce ) {  sotDebugTrace::openFile();  isDebugOnce = false; }
    // else { if(sotDEBUGFLOW.outputbuffer.good()) sotDebugTrace::closeFile(); }
    //if(sotDEBUGFLOW.outputbuffer.good()) { sotDebugTrace::closeFile();sotDebugTrace::openFile(); }
    sotDEBUGIN(15);
    /*
     * foreach stage: stage.initCOD(Ir_init)
     * u = 0
     * u0 = solve
     * do
     *   tau,cst_ref = max( violation(stages) )
     *   u += (1-tau)u0 + tau*u1
     *   if( tau<1 )
     *     update(cst_ref); break;
     *
     *   lambda,w = computeLambda
     *   cst_ref,lmin = min( lambda,w )
     *   if lmin<0
     *     downdate( cst_ref )
     *
     */

    assert(VectorXi::LinSpaced(3,0,2)[0] == 0
	   && VectorXi::LinSpaced(3,0,2)[1] == 1
	   && VectorXi::LinSpaced(3,0,2)[2] == 2
	   && "new version of Eigen might have change the "
	   "order of arguments in LinSpaced, please correct");

    /*struct timeval t0,t1,t2;double time1,time2;
    gettimeofday(&t0,NULL);*/
    initialize();
    Y.computeExplicitly(); // TODO: this should be done automatically on Y size.
    /*gettimeofday(&t1,NULL);
    time1 = ((t1.tv_sec-t0.tv_sec)+(t1.tv_usec-t0.tv_usec)/1.0e6);*/

    int iter = 0;
    unsigned int stageMinimal = 0;
    do
      {
	iter ++; sotDEBUG(5) << " --- *** \t" << iter << "\t***.---" << std::endl;
	//if( iter>1 ) { break; }

	if( sotDEBUG_ENABLE(15) )  show( sotDEBUGFLOW );
	assert( testRecomposition(&std::cerr) );
	damp();
	computeSolution();
	assert( testSolution(&std::cerr) );

	double tau = computeStepAndUpdate();
	if( tau<1 )
	  {
	    sotDEBUG(5) << "Update done, make step <1." << std::endl;
	    makeStep(tau);
	  }
	else
	  {
	    sotDEBUG(5) << "No update, make step ==1." << std::endl;
	    makeStep();

	    for( ;stageMinimal<=stages.size();++stageMinimal )
	      {
		sotDEBUG(5) << "--- Started to examinate stage " << stageMinimal << std::endl;
		computeLagrangeMultipliers(stageMinimal);
		if( sotDEBUG_ENABLE(15) )  show( sotDEBUGFLOW );
		//assert( testLagrangeMultipliers(stageMinimal,std::cerr) );

		if( searchAndDowndate(stageMinimal) )
		  {
		    sotDEBUG(5) << "Lagrange<0, downdate done." << std::endl;
		    break;
		  }

		for( unsigned int i=0;i<stageMinimal;++i )
		  stages[i]->freezeSlacks(false);
		if( stageMinimal<nbStages() )
		  stages[stageMinimal]->freezeSlacks(true);

	      }
	  }

	if( iter>1000 ) throw 666;
    } while(stageMinimal<=nbStages());
    sotDEBUG(5) << "Lagrange>=0, no downdate, active search completed." << std::endl;
    /*gettimeofday(&t2,NULL);
    time2 = ((t2.tv_sec-t1.tv_sec)+(t2.tv_usec-t1.tv_usec)/1.0e6);
    std::ofstream fup("/tmp/haset.dat",std::ios::app);
    fup << time1<<"\t"<<time2<<"\t"<<iter<<"\t";*/

    u=solution;
    sotDEBUG(5) << "uf =" << (MATLAB)u << std::endl;
    sotDEBUGOUT(15);
  }



  /* --- TESTS -------------------------------------------------------------- */
  /* --- TESTS -------------------------------------------------------------- */
  /* --- TESTS -------------------------------------------------------------- */
  bool HCOD::
  testRecomposition( std::ostream* os )
  {
    sotDEBUGPRIOR(+20);
    bool res = true;
    for( unsigned int i=0;i<stages.size();++i )
      {
	bool sres=stages[i]->testRecomposition();
	if( os&&(!sres) ) *os << "Stage " <<i<<" is not properly recomposed."<<std::endl;
	res&=sres;
      }
    return res;
  }

  bool HCOD::
  testSolution( std::ostream* os )
  {
    sotDEBUGPRIOR(+20);
    bool res = true;
    for( unsigned int i=0;i<stages.size();++i )
      {
	bool sres=stages[i]->testSolution( uNext );
	if( os&&(!sres) ) *os << "Stage " <<i<<" has not been properly inverted."<<std::endl;
	res&=sres;
      }
    return res;



  }

  /* Compute sum(i=1:sr) Ji' li, with Jsr = I and lsr = u, and check
   * that the result is null. */
  bool HCOD::
  testLagrangeMultipliers( int unsigned stageRef,std::ostream* os ) const
  {
    assert( stageRef<=stages.size() );
    VectorXd verifL(sizeProblem);

    /* verifL = Jsr' lsr, with Jsr = I and lsr = u. */
    if( stageRef==stages.size() )
      { verifL = solution; }
    else
      verifL.setZero();

    /* verif += sum Ji' li. */
    const unsigned int nbstage = std::min((unsigned int)(stages.size()-1),stageRef);
    for( unsigned int i=0;i<=nbstage;++i )
      {
	const Stage & s = *stages[i];
	sotDEBUG(5) << "verif = " << (soth::MATLAB)verifL << std::endl;
	sotDEBUG(5) << "J = " << (soth::MATLAB)s.Jactive() << std::endl;
	sotDEBUG(5) << "l = " << (soth::MATLAB)s.getLagrangeMultipliers() << std::endl;
	verifL += s.Jactive().transpose()*s.getLagrangeMultipliers();
	sotDEBUG(5) << "verif = " << (soth::MATLAB)verifL << std::endl;
      }
    sotDEBUG(5) << "verif = " << (soth::MATLAB)verifL << std::endl;

    // Damping
    if( nbstage>0 && stageRef<stages.size() )
      {
      const Stage & s = *stages[nbstage];
      if( s.useDamp() )// && s.sizeN()>0 )
    	{
    	  const unsigned int sm = s.getSizeM();
    	  VectorXd z(sizeProblem); z.head(sm) = Ytu.head(sm);
    	  z.tail(sizeProblem-sm).setZero();
    	  VectorXd Yz(sizeProblem);
    	  Y.multiply(z,Yz);
    	  Yz *= s.damping()*s.damping();
    	  verifL += Yz;
    	}
    }
    sotDEBUG(5) << "verif = " << (soth::MATLAB)verifL << std::endl;

    const double sumNorm = verifL.norm();
    const bool res = sumNorm<1e-6;
    if(os&&(!res))
      (*os) << "TestLagrangian failed: norm is " << sumNorm << "."<<std::endl;

    return res;
  }

  void HCOD::
  show( std::ostream& os, bool check )
  {
    sotDEBUGIN(15);
    for( unsigned int i=0;i<stages.size();++i )
      {
	stages[i]->show(os,i+1,check);
      }

    MatrixXd Yex(sizeProblem,sizeProblem); Yex.setIdentity();
    Y.applyThisOnTheLeft(Yex);
    os<<"Y = " << (MATLAB)Yex << std::endl;
    if( isSolutionCpt )
      {
	os << "u0 = " << (MATLAB)solution << std::endl;
	os << "u1 = " << (MATLAB)uNext << std::endl;
	os << "Ytu0 = " << (MATLAB)Ytu << std::endl;
	os << "Ytu1 = " << (MATLAB)YtuNext << std::endl;
	assert( (solution-Y.matrixExplicit*Ytu).norm() < Stage::EPSILON );
	assert( (uNext-Y.matrixExplicit*YtuNext).norm() < Stage::EPSILON );
      }
    sotDEBUGOUT(15);
  }

  void HCOD::
  showActiveSet( std::ostream& os ) const
  {
    sotDEBUGIN(15);
    os << "{" << std::endl;
    for( unsigned int i=0;i<stages.size();++i )
      {
	os<< "    "; stages[i]->showActiveSet(os); os << std::endl;
      }
    os << "}" << std::endl;
    sotDEBUGOUT(15);
  }






} // namespace soth

