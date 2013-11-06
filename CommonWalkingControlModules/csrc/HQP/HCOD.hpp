#ifndef __SOTH_HCOD__
#define __SOTH_HCOD__

#include "soth/api.hpp"
#include "soth/Stage.hpp"
#include "soth/BaseY.hpp"
#include <boost/smart_ptr.hpp>

namespace soth
{


  class SOTH_EXPORT HCOD
  {
  protected:
    typedef boost::shared_ptr<soth::Stage> stage_ptr_t;
    typedef std::vector<stage_ptr_t> stage_sequence_t;
    typedef stage_sequence_t::iterator stage_iter_t;
    typedef stage_sequence_t::const_iterator stage_citer_t;
    typedef stage_sequence_t::reverse_iterator stage_riter_t;

  public:
    HCOD( unsigned int sizeProblem, unsigned int nbStage = 0 );

    void pushBackStage( const MatrixXd & J, const VectorBound & bounds );
    void pushBackStage( const unsigned int & nr, const double * Jdata, const Bound * bdata );
    void pushBackStage( const unsigned int & nr, const double * Jdata );
    void pushBackStages( const std::vector<MatrixXd> & J,
			 const std::vector<VectorBound> & bounds );

    Stage& stage( unsigned int i );
    const Stage& stage( unsigned int i ) const;
    inline Stage& operator[] ( unsigned int i ) { return stage(i); }
    inline const Stage& operator[] ( unsigned int i ) const { return stage(i); }

    void setInitialActiveSet();
    void setInitialActiveSet( const cstref_vector_t& Ir0,unsigned int k );
    cstref_vector_t getOptimalActiveSet( unsigned int k );
    std::vector<cstref_vector_t>   getOptimalActiveSet();
    void setInitialActiveSet( const  std::vector<cstref_vector_t> & Ir);

    void useDamp( bool c ) { withDamp = c ; }
    bool useDamp() const { return withDamp; }
    void setDamping( const double & d );
    double getMaxDamping() const;

    //sizes
    int sizeA() const;
    int rank() const;
    unsigned int nbStages() const { return stages.size(); }

    /* --- Decomposition --- */
  public:
    void reset( void );
    void initialize( void );
    void update( const unsigned int & stageUp,const ConstraintRef & cst );
    void update( stage_iter_t stageIter,const ConstraintRef & cst );
    void downdate( const unsigned int & stageDown, const unsigned int & row );
    void downdate( stage_iter_t stageIter,const unsigned int & row );
  protected:
    void updateY( const GivensSequence& Yup );

    /* --- Computations --- */
  public:
    void damp( void );
    void computeSolution( bool compute_u = true );
    void computeLagrangeMultipliers( const unsigned int & stageRef );
    double computeStepAndUpdate( void );
    double computeStep( void );
    bool searchAndDowndate( const unsigned int & stageRef );
    bool search( const unsigned int & stageRef );

    void makeStep( double tau, bool compute_u = true );
    void makeStep( bool compute_u = true );



    /* --- The big one --- */
  public:
    //template< typename VectorGen >
    void activeSearch( VectorXd & u );

    /* --- Tests --- */
  public:
    void show( std::ostream& os, bool check=false );
    void showActiveSet( std::ostream& os ) const;
    bool testRecomposition( std::ostream* os=NULL );
    bool testSolution( std::ostream* os=NULL );
    bool testLagrangeMultipliers( unsigned int stageRef,std::ostream* os=NULL ) const;
    bool testLagrangeMultipliers( unsigned int stageRef,std::ostream& os ) const
    { return testLagrangeMultipliers(stageRef,&os); }

    void setNameByOrder( const std::string root = ""  );
    void notifiorRegistration( const Stage::listener_function_t & f,
			       int stageRank = -1 );

    bool isDebugOnce;
    void debugOnce(std::string filename="",bool keepOpen = false);

  protected:
    HCOD( void ) : Y(0) {};

  protected:public://DEBUG
    unsigned int sizeProblem;
    soth::BaseY Y;
    stage_sequence_t stages;
    VectorXd solution;

    VectorXd uNext,Ytu,YtuNext,rho;
    int freezedStages;
    bool isReset,isInit,isSolutionCpt,withDamp;
  };



} // namespace soth


#endif // #ifndef __SOTH_HCOD__
