#include "soth/BaseY.hpp"

namespace soth
{
  // Empty construction with memory allocation.
  BaseY::BaseY( const unsigned int & insize )
    :isExplicit(false)
    ,size(insize)
    ,rank(0)
    ,matrixExplicit(size,size)
    ,householderEssential(size,size)
  {
    householderEssential.setZero();
  }

  void BaseY::
  computeExplicitly()
  {
    isExplicit = true;
    matrixExplicit = getHouseholderSequence();
    //std::cout << matrixExplicit << std::endl; exit(0);
  }

  BaseY& BaseY::operator*= (const Givens& g)
  {
    assert(isExplicit && "you can't add Givens rotation to Y if the householder rotation matrix has not been explicitly computed");
    g.applyThisOnTheLeft(matrixExplicit);
    return *this;
  }

  BaseY& BaseY::operator*= (const GivensSequence& G)
  {
    assert(isExplicit && "you can't add Givens rotation to Y if the householder rotation matrix has not been explicitly computed");
    G.applyThisOnTheLeft(matrixExplicit);
    return *this;
  }


  //// Y *= Yup. Carefull: there is some recopy here.
  //void BaseY::
  //composeOnTheRight( const BaseY& Yp )
  //{
  //  /* TODO */
  //  throw "TODO";
  //}
  //// Y *= HHup.

  
 // void BaseY::
 // composeOnTheRight( const HouseholderSequence & hh )
 // {
 //   if( isExplicit )
 //     {
	///* TODO */
	//throw "TODO";
 //     }
 //   else
 //     {
	//matrixHH.append(hh);
 //     }

 // }

} // namespace soth
