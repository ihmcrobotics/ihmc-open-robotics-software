package us.ihmc.robotics.math.trajectories.core;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

public class PolynomialFactory
{

   public static PolynomialBasics constructFromComplexPairRoot(ComplexNumber oneComplexRoot)
   {
      double a = oneComplexRoot.real();
      double b = oneComplexRoot.imag();

      return new Polynomial(new double[] {1.0, -2.0 * a, a * a + b * b}, false);
   }

   public static PolynomialBasics constructFromRealRoot(double realRoot)
   {
      return new Polynomial(new double[] {1.0, -realRoot}, false);
   }

   public static PolynomialBasics constructFromScaleFactorAndRoots(double scaleFactor, double[] realRoots, ComplexNumber[] complexRootPairs)
   {
     Polynomial scalePolynomial = new Polynomial(new double[] {scaleFactor});

      if (complexRootPairs == null)
         complexRootPairs = new ComplexNumber[] {};
      if (realRoots == null)
         realRoots = new double[] {};

      PolynomialBasics[] complexRootPolynomials = new Polynomial[complexRootPairs.length];
      PolynomialBasics[] realRootPolynomials = new Polynomial[realRoots.length];

      for (int i = 0; i < realRoots.length; i++)
      {
         realRootPolynomials[i] = PolynomialFactory.constructFromRealRoot(realRoots[i]);
      }

      for (int i = 0; i < complexRootPairs.length; i++)
      {
         complexRootPolynomials[i] = PolynomialFactory.constructFromComplexPairRoot(complexRootPairs[i]);
      }

      PolynomialBasics polynomialToReturn = scalePolynomial;

      for (PolynomialBasics polynomial : realRootPolynomials)
      {
         polynomialToReturn = times(polynomialToReturn, polynomial);
      }

      for (PolynomialBasics polynomial : complexRootPolynomials)
      {
         polynomialToReturn = times(polynomialToReturn, polynomial);
      }

      return polynomialToReturn;
   }

   public static PolynomialBasics copyAndScale(double multiplier, PolynomialReadOnly other)
   {
      double[] coefficients = new double[other.getMaximumNumberOfCoefficients()];

      for (int cIndex = 0; cIndex < other.getMaximumNumberOfCoefficients(); cIndex++)
      {
         coefficients[cIndex] = other.getCoefficient(cIndex) * multiplier;
      }

      return new Polynomial(coefficients);
   }

   public static PolynomialBasics times(PolynomialReadOnly polynomialA, PolynomialReadOnly polynomialB)
   {
      // Do convolution on the coefficients:

      int order = polynomialA.getNumberOfCoefficients() + polynomialB.getNumberOfCoefficients() - 2;

      double[] coefficients = new double[order + 1];

      for (int cIndex = 0; cIndex <= order; cIndex++)
      {
         coefficients[cIndex] = 0.0;

         for (int aIndex = 0; aIndex <= cIndex; aIndex++)
         {
            int bIndex = cIndex - aIndex;

            if ((aIndex >= 0) && (bIndex >= 0) && (aIndex < polynomialA.getNumberOfCoefficients()) && (bIndex < polynomialB.getNumberOfCoefficients()))
            {
               coefficients[cIndex] += polynomialA.getCoefficient(aIndex) * polynomialB.getCoefficient(bIndex);
            }
         }
      }

      return new Polynomial(coefficients);
   }
}
