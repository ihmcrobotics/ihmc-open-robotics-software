#include <assert.h>
#include <cmath>
#include "soth/Bound.hpp"

namespace soth
{
  Bound::Bound( void )
    : type(BOUND_NONE),valInf(0),valSup(0),valTwin(valInf)
  {
  }

  Bound::Bound( const Bound & clone )
    : type(clone.type),valInf(clone.valInf),valSup(clone.valSup),valTwin(valInf)
  {
  }

  Bound::Bound( const double & val, bound_t inType )
    : type(inType),valInf(val),valSup(val),valTwin(valInf)
  {
    assert( inType != BOUND_DOUBLE );
    assert( inType != BOUND_NONE );

    type = inType;
  }

  Bound::Bound( const double & inValInf, const double & inValSup )
    : type(BOUND_DOUBLE),valInf(inValInf),valSup(inValSup),valTwin(valInf)
  {
  }
  Bound::Bound( const double & val )
    : type(BOUND_TWIN),valInf(val),valSup(val),valTwin(valInf)
  {
  }

  const double& Bound::
  getBound( bound_t inType ) const
  {
    switch( inType )
      {
      case BOUND_INF:
	assert( (type==BOUND_INF)||(type==BOUND_DOUBLE) );
	return valInf;
      case BOUND_SUP:
	assert( (type==BOUND_SUP)||(type==BOUND_DOUBLE) );
	return valSup;
      case BOUND_TWIN:
	assert(type==BOUND_TWIN);
	return valTwin;
      case BOUND_DOUBLE:
      case BOUND_NONE:
	;
      }
    assert( false&& "Cannot get a bound for 0 or double." );
    return valTwin;
  }

  /* Return the bound that is violated, NONE if bound are OK.
   * In case of twin-bounds, no check is performed, NONE is always returned. */
  Bound::bound_t Bound::
  check( const double & val,const double & EPSILON ) const
  {
    switch( type )
      {
      case BOUND_INF:
	if( val<valInf-EPSILON ) return BOUND_INF;
	break;
      case BOUND_SUP:
	if( valSup+EPSILON<val ) return BOUND_SUP;
	break;
      case BOUND_TWIN:
	break;
      case BOUND_DOUBLE:
	if( val<valInf-EPSILON ) return BOUND_INF;
	if( valSup+EPSILON<val ) return BOUND_SUP;
	break;
      case BOUND_NONE:
	assert( false&& "Cannot check a bound for 0 constraint." );
      }
    return BOUND_NONE;
  }

  /* Return the bound that is violated, NONE if bound are OK.
   * In case of twin-bounds, no check is performed, NONE is always returned. */
  Bound::bound_t Bound::
  check( const double & val,std::pair<double,double> damp, const double & EPSILON ) const
  {
    switch( type )
      {
      case BOUND_INF:
	if( val<valInf-damp.first-EPSILON ) return BOUND_INF;
	break;
      case BOUND_SUP:
	if( valSup+EPSILON+damp.second<val ) return BOUND_SUP;
	break;
      case BOUND_TWIN:
	break;
      case BOUND_DOUBLE:
	if( val<valInf-damp.first-EPSILON ) return BOUND_INF;
	if( valSup+damp.first+EPSILON<val ) return BOUND_SUP;
	break;
      case BOUND_NONE:
	assert( false&& "Cannot check a bound for 0 constraint." );
      }
    return BOUND_NONE;
  }

  /* Return the bound that is violated, NONE if bound are OK.
   * In case of twin-bounds, no check is performed, NONE is always returned. */
  Bound::bound_t Bound::
  checkSaturation( const double & val,const double & EPSILON ) const
  {
    switch( type )
      {
      case BOUND_INF:
	if( std::abs(val-valInf)<EPSILON ) return BOUND_INF;
	break;
      case BOUND_SUP:
	if( std::abs(val-valSup)<EPSILON ) return BOUND_SUP;
	break;
      case BOUND_TWIN:
	break;
      case BOUND_DOUBLE:
	if( std::abs(val-valInf)<EPSILON ) return BOUND_INF;
	if( std::abs(val-valSup)<EPSILON ) return BOUND_SUP;
	break;
      case BOUND_NONE:
	assert( type!=BOUND_NONE );
      }
    return BOUND_NONE;
  }

  double Bound::distance( const double & val ) const
  {
    double res=-1;
    switch( type )
      {
      case BOUND_INF:
	if( val>valInf ) res=0; else res=valInf-val;
	break;
      case BOUND_SUP:
	if( val<valSup ) res=0; else res=val-valSup;
	break;
      case BOUND_TWIN:
	res = std::abs( val-valTwin );
	break;
      case BOUND_DOUBLE:
	if( val>valInf )
	  if( val<valSup ) res=0;
	  else res=val-valSup;
	else res=valInf-val;
	break;
      case BOUND_NONE:
	assert( type!=BOUND_NONE );
	break;
     }
    assert(res>=0);
    return res;
  }

  Bound& Bound::operator= ( const Bound& clone )
  {
    valInf=clone.valInf;
    valSup=clone.valSup;
    type=clone.type;
    return *this;
  }

  Bound& Bound::operator= ( const double & val)
  {
    valTwin = val;
    type= BOUND_TWIN;
    return *this;
  }

  Bound& Bound::operator= ( const std::pair<double,double> & val)
  {
    valInf = val.first; valSup = val.second;
    type= BOUND_DOUBLE;
    return *this;
  }


  std::ostream& operator<< (std::ostream& os, const Bound&b )
  {
    switch( b.type )
      {
      case Bound::BOUND_INF:
	os << "[ " << b.valInf << ", +inf ]";
	break;
      case Bound::BOUND_SUP:
	os << "[ -inf ," << b.valSup << " ]";
	break;
      case Bound::BOUND_TWIN:
	os << "[ " << b.valTwin <<", " << b.valTwin << " ]";
	break;
      case Bound::BOUND_DOUBLE:
	os << "[ " << b.valInf <<", " << b.valSup << " ]";
	break;
      case Bound::BOUND_NONE:
	os << "[ -inf, +inf ]";
      }
    return os;
  }
  // std::ostream& operator<< (std::ostream& os, const bound_vector_t& t)
  // {
  //   os << "[ ";
  //   for( bound_vector_t::const_iterator iter=t.begin();
  // 	 iter!=t.end();++iter )
  //     {
  // 	os << *iter;
  // 	if( iter+1 != t.end() ) os <<"; "; else os<<" ]";
  //     }
  //   return os;
  // }

  std::ostream& operator<<( std::ostream&os,const ConstraintRef& cst )
  {
    switch( cst.type )
      {
      case Bound::BOUND_INF: os << "-"; break;
      case Bound::BOUND_SUP: os << "+"; break;
      case Bound::BOUND_DOUBLE: os << "+/-"; break;
      case Bound::BOUND_TWIN: os << "="; break;
      case Bound::BOUND_NONE: os << "(o)"; break;
      }
    return os << cst.row;
  }

  const ConstraintRef CONSTRAINT_VOID;//(-1,Bound::BOUND_NONE);


} // namespace soth
