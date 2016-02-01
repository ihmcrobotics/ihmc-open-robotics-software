#ifndef __SOTH_BOUND__
#define __SOTH_BOUND__

#include <Eigen/Core>
#include <iostream>
#include <vector>

#include <soth/api.hpp>

namespace soth
{
  class SOTH_EXPORT Bound
  {
  public:
    enum bound_t
      {
	BOUND_NONE
	,BOUND_INF
	,BOUND_SUP
	,BOUND_DOUBLE
	,BOUND_TWIN // equality
      };

  protected:
    bound_t type;
    // In case of twin bounds, the value is stored in valInf.
    double valInf,valSup;
    double & valTwin;

  public:
    Bound( void );
    Bound( const Bound& clone );
    Bound( const double & val, bound_t type );
    Bound( const double & inValInf, const double & inValSup );
    Bound( const double & valTwin );

    const bound_t& getType( void ) const { return type; }
    const double& getBound( bound_t type ) const;
    /* Return the bound that is violated, NONE if bound are OK.
     * In case of twin-bounds, no check is performed, NONE is always returned. */
    bound_t check( const double & val,const double & EPSILON=0 ) const;
    /* Return the bound that is violated, NONE if bound are OK.
     * In case of twin-bounds, no check is performed, NONE is always returned. */
    bound_t check( const double & val,std::pair<double,double> damp,
		   const double & EPSILON=0 ) const;
    /* Return the bound b s.t. |b-val|<EPSILON, and NONE if none. */
    bound_t checkSaturation( const double & val, const double & EPSILON  ) const;
    /* Return the distance to the bounds, 0 if satisfy, and real distance in the TWIN case. */
    double distance( const double & val ) const;

    Bound& operator= ( const Bound& clone );
    Bound& operator= ( const double & val);
    Bound& operator= ( const std::pair<double,double> & val);
    SOTH_EXPORT friend std::ostream& operator<< (std::ostream& os, const Bound& );

  }; // Class Bound

  typedef Eigen::Matrix<Bound, Eigen::Dynamic,1> VectorBound;
  // typedef std::pair<int,Bound::bound_t> ConstraintRef;
  struct ConstraintRef
  {
    int row;
    Bound::bound_t type;
    ConstraintRef( int r, Bound::bound_t t ) :row(r),type(t) {}
    ConstraintRef( void ): row(-1),type(Bound::BOUND_NONE) {}
    double sign() const { return (type==Bound::BOUND_SUP)?+1:-1; }
  };
  extern const ConstraintRef  CONSTRAINT_VOID;
  typedef std::vector<ConstraintRef> cstref_vector_t;

  SOTH_EXPORT std::ostream& operator<< (std::ostream& os, const VectorBound& t);
  SOTH_EXPORT std::ostream& operator<<( std::ostream&os,const ConstraintRef& cst );

} // namespace soth

#endif // #ifndef __SOTH_BOUND__
