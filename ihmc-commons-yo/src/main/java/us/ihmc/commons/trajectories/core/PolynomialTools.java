package us.ihmc.commons.trajectories.core;

import org.ejml.data.DMatrixRMaj;

public class PolynomialTools
{
   public static final boolean DEBUG = false;

   /**
    * Sets the given array to be:
    * <br> [1, x, x<sup>2</sup>, ..., x<sup>N</sup>]
    * <br> where N+1 is the length of the given array
    *
    * @param xPowers vector to set
    * @param x base of the power series
    */
   public static void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
      {
         xPowers[i] = xPowers[i - 1] * x;
      }
   }

   /**
    * Returns the constant coefficient at the exponent-th entry of the order-th derivative vector
    *  Example: order = 4, exponent = 5 ==> returns 5*4*3*2
     */
   public static int getDerivativeCoefficient(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
      {
         coeff *= i;
      }
      return coeff;
   }


   /**
    * Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the
    * YoPolynomials order-th derivative at x)
    *
    * @param order
    * @param x
    * @return
    */
   public static void getXPowersDerivativeVector(double[] xPowersToPack, DMatrixRMaj xPowersDerivativeVectorToPack, int order, double x, int numberOfCoefficients)
   {
      PolynomialTools.setXPowers(xPowersToPack, x);
      xPowersDerivativeVectorToPack.zero();

      int derivativeCoefficient = 0;
      for (int i = order; i < numberOfCoefficients; i++)
      {
         derivativeCoefficient = PolynomialTools.getDerivativeCoefficient(order, i);
         xPowersDerivativeVectorToPack.set(i, derivativeCoefficient * xPowersToPack[i - order]);
      }
   }
}
