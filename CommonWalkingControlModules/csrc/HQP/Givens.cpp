#include "soth/Givens.hpp"

namespace soth
{
  Givens::Givens()
    :i(0), j(0)
  {
  }

  Givens::Givens(double a, double b, int i, int j, double* z)
    :i(i), j(j)
  {
    makeGivens(a,b,i,j,z);
  }

  void Givens::makeGivens(double a, double b, int i, int j, double* z)
  {
    G.makeGivens(a,b,z);
    this->i = i;
    this->j = j;
    Gt = G.adjoint();
  }




  GivensSequence& GivensSequence::
  push(const Givens& g)
  {
    G.push_back(g);
    return *this;
  }

}
