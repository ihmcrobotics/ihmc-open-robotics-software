package us.ihmc.math;

import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.commons.trajectories.core.PolynomialMath;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;
import us.ihmc.commons.trajectories.interfaces.PolynomialReadOnly;

public class ComplexPolynomialTools
{
   /**
    * Performs the polynomial expansion at value X, computing f(x), where X is a complex number
    *
    * @param polynomial polynomial containing coefficients to compute
    * @param input      value at which to evaluate f(X) (x in the above equation).
    * @return f(X)
    */
   public static ComplexNumber evaluate(PolynomialReadOnly polynomial, ComplexNumber input)
   {
      ComplexNumber x_n = new ComplexNumber(1.0, 0.0);
      ComplexNumber ret = new ComplexNumber(0.0, 0.0);

      for (int i = 0; i < polynomial.getNumberOfCoefficients(); i++)
      {
         double coefficient = polynomial.getCoefficient(i);
         ret = ret.plus(x_n.times(coefficient));
         x_n = x_n.times(input);
      }

      return ret;
   }

   public static Polynomial constructFromRealRoot(double realRoot)
   {
      return new Polynomial(-realRoot, 1.0);
   }

   public static Polynomial constructFromComplexPairRoot(ComplexNumber oneComplexRoot)
   {
      double a = oneComplexRoot.real();
      double b = oneComplexRoot.imaginary();

      return new Polynomial(a * a + b * b, -2.0 * a, 1.0);
   }

   public static PolynomialBasics constructFromScaleFactorAndRoots(double scaleFactor, double[] realRoots, ComplexNumber[] complexRootPairs)
   {
      Polynomial scalePolynomial = new Polynomial(scaleFactor);

      if (complexRootPairs == null)
         complexRootPairs = new ComplexNumber[] {};
      if (realRoots == null)
         realRoots = new double[] {};

      Polynomial[] complexRootPolynomials = new Polynomial[complexRootPairs.length];
      Polynomial[] realRootPolynomials = new Polynomial[realRoots.length];

      for (int i = 0; i < realRoots.length; i++)
      {
         realRootPolynomials[i] = ComplexPolynomialTools.constructFromRealRoot(realRoots[i]);
      }

      for (int i = 0; i < complexRootPairs.length; i++)
      {
         complexRootPolynomials[i] = ComplexPolynomialTools.constructFromComplexPairRoot(complexRootPairs[i]);
      }

      PolynomialBasics polynomialToReturn = scalePolynomial;

      for (Polynomial polynomial : realRootPolynomials)
      {
         polynomialToReturn = PolynomialMath.times(polynomialToReturn, polynomial);
      }

      for (Polynomial polynomial : complexRootPolynomials)
      {
         polynomialToReturn = PolynomialMath.times(polynomialToReturn, polynomial);
      }

      return polynomialToReturn;
   }
}
