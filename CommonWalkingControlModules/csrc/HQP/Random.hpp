/* -------------------------------------------------------------------------- *
 * 
 * Random matrix generator.
 * 
 * -------------------------------------------------------------------------- */

#include "soth/Algebra.hpp"

namespace soth
{
  class Random
  {
    static const unsigned int SOTH_RND_MAX;// = 4278255361U;
    static const unsigned int MULT = 1884103651;
    static unsigned int current;

    static unsigned int next();

  public:
    static void setSeed(unsigned int newSeed);

    template <typename ReturnType>
      static ReturnType rand();

    template <typename ReturnType>
      static ReturnType randMax();
  };


  template <> inline unsigned int Random::rand() {return next();}
  template <> inline unsigned int Random::randMax() {return SOTH_RND_MAX;}
  template <> inline int Random::rand() {return next()>>1;}
  template <> inline int Random::randMax() {return SOTH_RND_MAX>>1;}
  template <> inline double Random::rand() {return static_cast<double>(next())/(SOTH_RND_MAX);}
  template <> inline double Random::randMax() {return 1.;}


  template<typename Scalar> 
    struct rnd_traits
    {
      static Scalar max() {return Random::randMax<Scalar>();}
      static Scalar default_min() {return Scalar(-1);}
      static Scalar default_max() {return Scalar(1);}
    };

  template<>
    struct rnd_traits<int>
    {
      static int max() {return Random::randMax<int>();}
      static int default_min() {return -10;}
      static int default_max() {return 11;}
    };

  class MatrixRnd
  {
  public:
    template<typename Derived>
      static MatrixBase<Derived>&
      randomize(MatrixBase<Derived>& m,
		typename Derived::Scalar min= rnd_traits<typename Derived::Scalar>::default_min(),
		typename Derived::Scalar max= rnd_traits<typename Derived::Scalar>::default_max())
      {
	for ( typename Derived::Index i=0; i<m.rows(); ++i)
	  {
	    for ( typename Derived::Index j=0; j<m.cols(); ++j)
	      m(i,j) = static_cast<typename Derived::Scalar>((max-min)* Random::rand<double>()) + min;
	  }
	return m;
      }
  };


  // Simulate a white noise with mean 0 and var 1.
  double whiteNoise(void);
  // Simulate any white noise.
  int whiteNoise( int mean,double var );
  // Simulate a discrete uniform law inside [ bmin,bmax ] (each bound
  // being reached).
  int randu( int bmin,int bmax );


} // namespace soth.
