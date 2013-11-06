#ifndef __SOTH_ALGEBRA__
#define __SOTH_ALGEBRA__

#include <Eigen/Core>
#include <iostream>
#include "soth/api.hpp"

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

namespace soth
{
  //  USING_PART_OF_NAMESPACE_EIGEN;
  using namespace Eigen;

} // namespace soth




/* -------------------------------------------------------------------------- */
/* --- DEBUG ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

namespace soth
{
#define EI_FOREACH(a,b)  for( Index a=0;a<b.size();++a )

  struct MATLAB
  {
    SOTH_EXPORT friend std::ostream & operator << (std::ostream & os, const MATLAB & m );

    SOTH_EXPORT MATLAB( const double& x );
    template< typename Derived >
    MATLAB( const MatrixBase<Derived> & m1 ) { genericInit(m1); }
    template< typename Derived >
    MATLAB( const MatrixBase<Derived> & m1, bool id );
    template< typename Rotation >
    MATLAB( unsigned int size,const Rotation & m1 );

    template< typename Derived >
    void genericInit( const MatrixBase<Derived> & m1 );

    template< typename Derived >
    void initMatrix( const MatrixBase<Derived> & m1 );
    template< typename VectorGen >
    void initVector( const VectorGen & v );
    SOTH_EXPORT inline void initMatrixNull( void );
    SOTH_EXPORT inline void initMatrixColNull( unsigned int size );
    SOTH_EXPORT inline void initMatrixRowNull( unsigned int size );


    SOTH_EXPORT static bool fullPrec;
    SOTH_EXPORT static unsigned int precision;
    std::string str;
  };

} // namespace soth


// --- HEAVY CODE ---
namespace soth
{

  template< typename Derived >
  void MATLAB::genericInit( const MatrixBase<Derived> & m1 )
  {
    if( m1.rows()==0 ) initMatrixRowNull( m1.cols() );
    else if( m1.cols()==0 ) initMatrixColNull( m1.rows() );
    else if( m1.IsVectorAtCompileTime)
      {
	if( m1.cols()==1 ) initVector( m1.col(0) );
	else if( m1.rows()==1 ) initVector( m1.row(0) );
      }
    else initMatrix(m1);
  }


  template< typename VectorGen >
    void MATLAB::initVector( const VectorGen & m1 )
    {
      std::ostringstream os; os.precision(MATLAB::precision);
      os << "[ ";
      std::ostringstream ostmp; ostmp.precision(MATLAB::precision);
      for(int i=0;i<m1.size();++i )
	{
	  if( m1[i]<0 ) ostmp << "-"; else ostmp << " ";
	  if(MATLAB::fullPrec||(std::abs(m1[i])>1e-6)) ostmp <<  std::abs(m1[i]);
	  else { ostmp << "0"; }
	  if( m1.size()!=i+1 )
	    {
	      ostmp << ",";
	      const int size = ostmp.str().length();
	      for( unsigned int i=size;i<8;++i) ostmp<<" ";
	      ostmp << "\t";
	    }
	  os << ostmp.str(); ostmp.str("") ;
	}
      os << "  ]';";
      str = os.str();
    }

  template< typename Derived >
  MATLAB::MATLAB( const MatrixBase<Derived> & m1, bool id )
  {
    if( id )
      {
	std::ostringstream os;
	os << "eye(" << m1.rows() << "," << m1.cols() << ");";
	str = os.str();
      }
    else
      {	genericInit(m1);      }
  }

  void MATLAB::initMatrixNull( void ) { str = "[];"; }
  void MATLAB::initMatrixColNull( unsigned int size )
  {   std::ostringstream os;  os << "zeros("<<size<<",0);";  str = os.str();  }
  void MATLAB::initMatrixRowNull( unsigned int size )
  {   std::ostringstream os;  os << "zeros(0,"<<size<<");";  str = os.str();  }


  template< typename Derived >
    void MATLAB::initMatrix( const MatrixBase<Derived> & m1 )
    {
      std::ostringstream os; os.precision(MATLAB::precision);
      if( fullPrec ) { os << "[...\n" << m1 << "];"; str=os.str(); return; }
      os << "...\n[ ";
      std::ostringstream ostmp; ostmp.precision(MATLAB::precision);
      for(int i=0;i<m1.rows();++i )
	{
	  for(int j=0;j<m1.cols();++j )
	    {
	      if( m1(i,j)<0 ) ostmp <<"-"; else ostmp << " ";
	      if(MATLAB::fullPrec||(std::abs(m1(i,j))>1e-6)) ostmp <<  std::abs(m1(i,j));
	      else { ostmp << "0"; }
	      if( m1.cols()!=j+1 )
		{
		  ostmp << ",";
		  const int size = ostmp.str().length();
		  for( unsigned int i=size;i<8;++i) ostmp<<" ";
		  ostmp << "\t";
		}
	      os << ostmp.str(); ostmp.str("") ;
	    }
	  if( m1.rows()!=i+1 ) { os << " ;" << std::endl<<"  "; }
	  else { os << "  ];"; }
	}
      str = os.str();
    }


} // namespace soth





#endif // #ifndef __SOTH_ALGEBRA__
