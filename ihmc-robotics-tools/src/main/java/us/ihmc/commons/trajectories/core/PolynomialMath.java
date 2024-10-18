package us.ihmc.commons.trajectories.core;

import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;
import us.ihmc.commons.trajectories.interfaces.PolynomialReadOnly;

public class PolynomialMath
{
   /**
    * Computes C(x) = A(x) + B(x), where A(x) and B(x) are both polynomials
    * @param a first polynomial in the sum
    * @param b second polynomial in the sum
    * @return C(x)
    */
   public static Polynomial plus(PolynomialReadOnly a, PolynomialReadOnly b)
   {
      int newOrder = a.getOrder();
      if (b.getOrder() > newOrder)
         newOrder = b.getOrder();

      double[] newCoefficients = new double[newOrder + 1];

      for (int cIndex = 0; cIndex <= newOrder; cIndex++)
      {
         newCoefficients[cIndex] = 0.0;
         if (cIndex < a.getNumberOfCoefficients())
            newCoefficients[cIndex] += a.getCoefficient(cIndex);
         if (cIndex < b.getNumberOfCoefficients())
            newCoefficients[cIndex] += b.getCoefficient(cIndex);
      }

      return new Polynomial(newCoefficients);
   }

   /**
    * Performs the polynomial expansion at value X, computing f(x)
    * @param polynomial polynomial containing coefficients to compute
    * @param x value at which to evaluate f(X)
    * @return f(X)
    */
   public static double evaluate(PolynomialReadOnly polynomial, double x)
   {
      double x_n = 1.0;
      double ret = 0.0;

      for (int i = 0; i < polynomial.getNumberOfCoefficients(); i++)
      {
         double coefficient = polynomial.getCoefficient(i);
         ret += coefficient * x_n;
         x_n *= x;
      }

      return ret;
   }

   /**
    * Computes C(x) = A(x) * B(x), where A(x) and B(x) are both polynomials
    * @param a first polynomial in the product
    * @param b second polynomial in the product
    * @return C(x)
    */
   public static PolynomialBasics times(PolynomialReadOnly a, PolynomialReadOnly b)
   {
      // Do convolution on the coefficients:

      int order = a.getOrder() + b.getOrder();

      double[] coefficients = new double[order + 1];

      for (int cIndex = 0; cIndex <= order; cIndex++)
      {
         coefficients[cIndex] = 0.0;

         for (int aIndex = 0; aIndex <= cIndex; aIndex++)
         {
            int bIndex = cIndex - aIndex;

            if ((aIndex >= 0) && (bIndex >= 0) && (aIndex < a.getNumberOfCoefficients()) && (bIndex < b.getNumberOfCoefficients()))
            {
               coefficients[cIndex] += a.getCoefficient(aIndex) * b.getCoefficient(bIndex);
            }
         }
      }

      return new Polynomial(coefficients);
   }

   public static Polynomial times(PolynomialReadOnly polynomial, double multiplier)
   {
      double[] coefficients = new double[polynomial.getNumberOfCoefficients()];
      for (int i = 0; i < coefficients.length; i++)
         coefficients[i] = multiplier * polynomial.getCoefficient(i);
      return new Polynomial(coefficients);
   }

   
}
