package us.ihmc.robotics.dataStructures;

import static org.ejml.ops.CommonOps.solve;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.MathTools;

/**
 * <p>Polynomial </p>
 *
 * <p>Polynomial Function with real coefficients. Immuatable. </p>
 *
 * <p>Copyright (c) 2008</p>
 *
 * <p>IHMC-Yobotics </p>
 *
 * @author IHMC-Yobotics Biped Team
 * @version 1.0
 */
public class Polynomial
{
   private final double[] coefficients;
   private final double[] derivativeCoefficients;
   private final double[] doubleDerivativeCoefficients;
   private DenseMatrix64F constraintMatrix;
   private DenseMatrix64F constraintVector;
   private DenseMatrix64F coefficientVector;
   
   public Polynomial(double constant)
   {
      this(new double[] {constant});
   }

   public Polynomial(double coefficient1, double constant)
   {
      this(new double[] {coefficient1, constant});
   }

   public Polynomial(double coefficient2, double coefficient1, double constant)
   {
      this(new double[] {coefficient2, coefficient1, constant});
   }

   public Polynomial(double coefficient3, double coefficient2, double coefficient1, double constant)
   {
      this(new double[] {coefficient3, coefficient2, coefficient1, constant});
   }

   public Polynomial(double[] coefficientsHighOrderFirst)
   {
      if ((coefficientsHighOrderFirst == null) || (coefficientsHighOrderFirst.length < 1))
         throw new RuntimeException("(coefficientsHighOrderFirst == null) || (coefficientsHighOrderFirst.length < 1)");

      int coefficientsToSkip = 0;
      if ((coefficientsHighOrderFirst.length > 1) && (Math.abs(coefficientsHighOrderFirst[0]) < 1e-7))
      {
         // Skip any zero coefficients, that is any whose coefficient is < 1e-7 times the max coefficient.
         double maxCoefficient = findMaxAbsoluteCoefficient(coefficientsHighOrderFirst);

         for (coefficientsToSkip = 0; coefficientsToSkip < coefficientsHighOrderFirst.length; coefficientsToSkip++)
         {
            if (Math.abs(coefficientsHighOrderFirst[coefficientsToSkip]) > 1e-7 * maxCoefficient)
            {
               break;
            }
         }
      }

      if (coefficientsHighOrderFirst.length - coefficientsToSkip == 0)    // Skipped them all since was {0.0, 0.0, ..., 0.0};
      {
         coefficientsToSkip = coefficientsToSkip - 1;
      }

      this.coefficients = new double[coefficientsHighOrderFirst.length - coefficientsToSkip];

      for (int i = coefficientsToSkip; i < coefficientsHighOrderFirst.length; i++)
      {
         this.coefficients[i - coefficientsToSkip] = coefficientsHighOrderFirst[i];
      }
      
      if(coefficientsHighOrderFirst.length == 1)
      {
    	  this.derivativeCoefficients = new double[]{0.0};    	  
      }
      else
      {
          this.derivativeCoefficients = new double[coefficientsHighOrderFirst.length - 1];
          int length = coefficientsHighOrderFirst.length;
          for(int i = 0; i < coefficientsHighOrderFirst.length - 1; i++)
          {
        	  this.derivativeCoefficients[i] = (length - i - 1) * coefficientsHighOrderFirst[i];
          }
      }
      
      if(coefficientsHighOrderFirst.length < 3)
      {
    	  this.doubleDerivativeCoefficients = new double[]{0.0};    	  
      }
      else
      {
          this.doubleDerivativeCoefficients = new double[coefficientsHighOrderFirst.length - 2];
          int length = coefficientsHighOrderFirst.length;
          for(int i = 0; i < coefficientsHighOrderFirst.length - 2; i++)
          {
        	  this.doubleDerivativeCoefficients[i] = (length - i - 1) * (length - i - 2) * coefficientsHighOrderFirst[i];
          }
      }

      if (coefficients == null)
         throw new RuntimeException("(coefficients == null)");

      if (coefficients.length < 1)
      {
         System.err.println("coefficientsHighOrderFirst[0] = " + coefficientsHighOrderFirst[0]);
         System.err.println("coefficientsHighOrderFirst[1] = " + coefficientsHighOrderFirst[1]);

         throw new RuntimeException("(coefficients.length < 1)");

      }

   }
   
   private double findMaxAbsoluteCoefficient(double[] coefficients)
   {
      double maxCoefficient = 0.0;
      for (double coefficient : coefficients)
      {
         if (Math.abs(coefficient) > Math.abs(maxCoefficient))
         {
            maxCoefficient = Math.abs(coefficient);
         }
      }

      return maxCoefficient;
   }

   public static Polynomial constructFromComplexPairRoot(ComplexNumber oneComplexRoot)
   {
      double a = oneComplexRoot.real();
      double b = oneComplexRoot.imag();

      return new Polynomial(new double[] {1.0, -2.0 * a, a * a + b * b});
   }

   public static Polynomial constructFromRealRoot(double realRoot)
   {
      return new Polynomial(new double[] {1.0, -realRoot});
   }

   public static Polynomial constructFromScaleFactorAndRoots(double scaleFactor, double[] realRoots, ComplexNumber[] complexRootPairs)
   {
      Polynomial scalePolynomial = new Polynomial(new double[] {scaleFactor});

      if (complexRootPairs == null)
         complexRootPairs = new ComplexNumber[]
         {
         };
      if (realRoots == null)
         realRoots = new double[]
         {
         };

      Polynomial[] complexRootPolynomials = new Polynomial[complexRootPairs.length];
      Polynomial[] realRootPolynomials = new Polynomial[realRoots.length];

      for (int i = 0; i < realRoots.length; i++)
      {
         realRootPolynomials[i] = Polynomial.constructFromRealRoot(realRoots[i]);
      }

      for (int i = 0; i < complexRootPairs.length; i++)
      {
         complexRootPolynomials[i] = Polynomial.constructFromComplexPairRoot(complexRootPairs[i]);
      }

      Polynomial polynomialToReturn = scalePolynomial;

      for (Polynomial polynomial : realRootPolynomials)
      {
         polynomialToReturn = polynomialToReturn.times(polynomial);
      }

      for (Polynomial polynomial : complexRootPolynomials)
      {
         polynomialToReturn = polynomialToReturn.times(polynomial);
      }

      return polynomialToReturn;
   }

   public double evaluate(double input)
   {
      double x_n = 1.0;
      double ret = 0.0;

      for (int i = coefficients.length - 1; i >= 0; i--)
      {
         double coefficient = coefficients[i];
         ret = ret + coefficient * x_n;
         x_n = x_n * input;
      }

      return ret;
   }
   
   public double evaluateDerivative(double input)
   {
	   double x_n = 1.0;
	   double ret = 0.0;

	   for (int i = coefficients.length - 2; i >= 0; i--) 
	   {
		   double coefficient = derivativeCoefficients[i];
		   ret = ret + coefficient * x_n;
		   x_n = x_n * input;
	   }

	   return ret;
   }
   
   public double evaluateDoubleDerivative(double input)
   {
	   double x_n = 1.0;
	   double ret = 0.0;

	   for (int i = coefficients.length - 3; i >= 0; i--) 
	   {
		   double coefficient = doubleDerivativeCoefficients[i];
		   ret = ret + coefficient * x_n;
		   x_n = x_n * input;
	   }

	   return ret;
   }
   

   public ComplexNumber evaluate(ComplexNumber input)
   {
      ComplexNumber x_n = new ComplexNumber(1.0, 0.0);
      ComplexNumber ret = new ComplexNumber(0.0, 0.0);

      for (int i = coefficients.length - 1; i >= 0; i--)
      {
         double coefficient = coefficients[i];
         ret = ret.plus(x_n.times(coefficient));
         x_n = x_n.times(input);
      }

      return ret;
   }


   public int getOrder()
   {
      return coefficients.length - 1;
   }

   public double[] getCoefficients()
   {
      double[] ret = new double[coefficients.length];

      for (int i = 0; i < coefficients.length; i++)
      {
         ret[i] = coefficients[i];
      }

      return ret;
   }

   public Polynomial times(double multiplier)
   {
      double[] coefficients = new double[this.coefficients.length];

      for (int cIndex = 0; cIndex < this.coefficients.length; cIndex++)
      {
         coefficients[cIndex] = this.coefficients[cIndex] * multiplier;
      }

      return new Polynomial(coefficients);
   }

   public Polynomial times(Polynomial polynomialB)
   {
      // Do convolution on the coefficients:

      int order = this.getOrder() + polynomialB.getOrder();

      double[] coefficients = new double[order + 1];

      for (int cIndex = 0; cIndex <= order; cIndex++)
      {
         coefficients[cIndex] = 0.0;

         for (int aIndex = 0; aIndex <= cIndex; aIndex++)
         {
            int bIndex = cIndex - aIndex;

            if ((aIndex >= 0) && (bIndex >= 0) && (aIndex < this.coefficients.length) && (bIndex < polynomialB.coefficients.length))
            {
               coefficients[cIndex] += this.coefficients[aIndex] * polynomialB.coefficients[bIndex];
            }
         }
      }

      Polynomial ret = new Polynomial(coefficients);

      return ret;
   }

   public Polynomial plus(Polynomial polynomial)
   {
      int newOrder = this.getOrder();
      if (polynomial.getOrder() > newOrder)
         newOrder = polynomial.getOrder();

      // System.out.println("newOrder = " + newOrder);

      double[] newCoefficients = new double[newOrder + 1];

      for (int cIndex = 0; cIndex <= newOrder; cIndex++)
      {
         int thisIndex = this.getOrder() - cIndex;
         int otherIndex = polynomial.getOrder() - cIndex;
         int newIndex = newOrder - cIndex;

         newCoefficients[newIndex] = 0.0;
         if (thisIndex >= 0)
            newCoefficients[newIndex] += this.coefficients[thisIndex];
         if (otherIndex >= 0)
            newCoefficients[newIndex] += polynomial.coefficients[otherIndex];
      }

      return new Polynomial(newCoefficients);
   }


   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      for (int i = 0; i < coefficients.length - 1; i++)
      {
         builder.append(coefficients[i]);
         builder.append(" * x");
         int exponent = coefficients.length - i - 1;

         if (exponent > 1)
         {
            builder.append("^");
            builder.append(exponent);
         }

         builder.append(" + ");
      }

      builder.append(coefficients[coefficients.length - 1]);

      return builder.toString();
   }

   public boolean epsilonEquals(Polynomial polynomial, double epsilon)
   {
      if (coefficients.length != polynomial.coefficients.length)
         return false;

      for (int i = 0; i < coefficients.length; i++)
      {
         if (Math.abs(coefficients[i] - polynomial.coefficients[i]) > epsilon)
            return false;
      }

      return true;
   }
   
   public void setQuintic(double x0, double x1, double y0, double yd0, double ydd0, double y1, double yd1, double ydd1)
   {
	   MathTools.checkEquals(coefficients.length, 6);
	   constraintMatrix = new DenseMatrix64F(new double[6][6]);
	   constraintVector = new DenseMatrix64F(new double[6][1]);
	   coefficientVector = new DenseMatrix64F(new double[6][1]);

	   setPointConstraint(0, x0, y0);
	   setDerivativeConstraint(1, x0, yd0);
	   setDoubleDerivativeConstraint(2, x0, ydd0);

	   setPointConstraint(3, x1, y1);
	   setDerivativeConstraint(4, x1, yd1);
	   setDoubleDerivativeConstraint(5, x1, ydd1);
	   
	   solve(constraintMatrix, constraintVector, coefficientVector);
	   setVariables();
   }
   
   public void setCubic(double x0, double x1, double y0, double yd0, double y1, double yd1)
   {
	   MathTools.checkEquals(coefficients.length, 4);
	   constraintMatrix = new DenseMatrix64F(new double[4][4]);
	   constraintVector = new DenseMatrix64F(new double[4][1]);
	   coefficientVector = new DenseMatrix64F(new double[4][1]);

	   setPointConstraint(0, x0, y0);
	   setDerivativeConstraint(1, x0, yd0);

	   setPointConstraint(2, x1, y1);
	   setDerivativeConstraint(3, x1, yd1);
	   
	   solve(constraintMatrix, constraintVector, coefficientVector);
	   setVariables();
   }
   
   private void setPointConstraint(int row, double xValue, double yValue)
   {
	   double x_n = 1.0;
	   
	   for(int column = coefficients.length - 1; column >= 0; column--)
	   {
		   constraintMatrix.set(row, column, x_n);
		   x_n *= xValue;
	   }
	   
	   constraintVector.set(row, yValue);
   }
   
   private void setDerivativeConstraint(int row, double xValue, double yValue)
   {
	   double x_n = 1.0;
	   constraintMatrix.set(row, coefficients.length - 1, 0.0);
	   
	   for(int column = coefficients.length - 2; column >= 0; column--)
	   {
		   constraintMatrix.set(row, column, (coefficients.length - column - 1) * x_n);
		   x_n *= xValue;
	   }
	   
	   constraintVector.set(row, yValue);
   }

   private void setDoubleDerivativeConstraint(int row, double xValue, double yValue)
   {
	   double x_n = 1.0;
	   constraintMatrix.set(row, coefficients.length - 1, 0.0);
	   constraintMatrix.set(row, coefficients.length - 2, 0.0);
	   
	   for(int column = coefficients.length - 3; column >= 0; column--)
	   {
		   constraintMatrix.set(row, column, (coefficients.length - column - 1) * (coefficients.length - column - 2) * x_n);
		   x_n *= xValue;
	   }
	   
	   constraintVector.set(row, yValue);
   }
   
   private void setVariables()
   {  
      int length = coefficients.length;
      
	   for (int row = 0; row < length; row++)
	   {
		   coefficients[row]                 = coefficientVector.get(row, 0);
		   
		   if( row< length-1)
		      derivativeCoefficients[row]       = coefficientVector.get(row, 0);
		   
		   if( row< length-2)
		      doubleDerivativeCoefficients[row] = coefficientVector.get(row, 0) ;
	   }
	   
	   for(int i = 0; i < length - 1; i++)
      {
	      derivativeCoefficients[i] *= (length - i - 1) ;
      }
	   
	   for(int i = 0; i < length - 2; i++)
      {
	      doubleDerivativeCoefficients[i] *= (length - i - 1) * (length - i - 2);
      }
   }
   
   public boolean equalsZero()
   {
      return (Math.abs(this.coefficients[0]) < 1e-15);
   }
   
   public double[] getDerivativeCoefficients()
   {
	   return derivativeCoefficients.clone();
   }

   public double[] getDoubleDerivativeCoefficients()
   {
	   return doubleDerivativeCoefficients.clone();
   }
}
