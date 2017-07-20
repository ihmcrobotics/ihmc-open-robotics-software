package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoPolynomialTest
{
   private static double EPSILON = 1e-6;
   
   String namePrefix = "YoPolynomialTest";

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearDerivativePointManual()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoPolynomial linear = new YoPolynomial(namePrefix + "Linear", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      
      linear.setLinear(x0, xf, y0, yf);
      
      double x = 2.0/3.0 * (xf - x0);
      double a0 = linear.getCoefficient(0);
      double a1 = linear.getCoefficient(1);
      
      double yLinear = linear.getDerivative(0, x);
      double yManual = a0 + a1*x;
      assertEquals(yLinear, yManual, EPSILON);
            
      double dyLinear = linear.getDerivative(1, x);
      double dyManual = a1;
      assertEquals(dyLinear, dyManual, EPSILON);
      
      double ddyLinear = linear.getDerivative(2, x);
      double ddyManual = 0.0;
      assertEquals(ddyLinear, ddyManual, EPSILON); 
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearDerivativePointAutomated()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoPolynomial linear = new YoPolynomial(namePrefix + "Linear", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      
      linear.setLinear(x0, xf, y0, yf);
      
      double x = 2.0/3.0 * (xf - x0);

      compareDerivativesPoint(linear, x);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCubicDerivativePointAutomated()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      double x0 = 1.0, xf = 2.0;
      double y0 = 0.5, yf = 1.5;
      double dy0 = -0.5, dyf = 2.0;
            
      cubic.setCubic(x0, xf, y0, dy0, yf, dyf);
      
      double x = 2.0/3.0 * (xf - x0);

      compareDerivativesPoint(cubic, x);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testXPowersDerivativeVectorCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         double scaleDY0 = 1.0 / Math.random(), scaleDYf = 1.0 / Math.random();
         
         double x0 = Math.signum(Math.random()) * Math.random() * scaleX0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
         double dy0 = Math.signum(Math.random()) * Math.random() * scaleDY0, dyf = Math.signum(Math.random()) * Math.random() * scaleDYf;
               
         cubic.setCubic(x0, xf, y0, dy0, yf, dyf);
         
         double x = Math.random() * (xf - x0);
         
         compareXPowersDerivativesVector(cubic, x);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDerivativeCoefficients()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 8;
      YoPolynomial septic = new YoPolynomial(namePrefix + "Septic", numberOfCoefficients, registry);
      
      double x0 = 1.0, x1 = 1.2, x2 = 1.9, xf = 2.0;
      double y0 = 0.5, y1 = -0.75, y2 = 1.3, yf = 1.5;
      double dy0 = -0.5, dy1 = 0.5, dy2 = 1.0, dyf = 2.0;
            
      septic.setSeptic(x0, x1, x2, xf, y0, dy0, y1, dy1, dy2, dy2, yf, dyf);
      
      int order3Exponent1Func = septic.getDerivativeCoefficient(3, 1);
      int order3Exponent1Hand = 0;
      assertEquals(order3Exponent1Func, order3Exponent1Hand, EPSILON);
      
      int order6Exponent7Func = septic.getDerivativeCoefficient(6, 7);
      int order6Exponent7Hand = 5040;
      assertEquals(order6Exponent7Func, order6Exponent7Hand, EPSILON);
      
      int order0Exponent5Func = septic.getDerivativeCoefficient(0, 5);
      int order0Exponent5Hand = 1;
      assertEquals(order0Exponent5Func, order0Exponent5Hand, EPSILON);
      
      int order3Exponent4Func = septic.getDerivativeCoefficient(3, 4);
      int order3Exponent4Hand = 24;
      assertEquals(order3Exponent4Func, order3Exponent4Hand, EPSILON);
      
      int order5Exponent2Func = septic.getDerivativeCoefficient(5, 2);
      int order5Exponent2Hand = 0;
      assertEquals(order5Exponent2Func, order5Exponent2Hand, EPSILON);
      
      int order1Exponent5Func = septic.getDerivativeCoefficient(1, 5);
      int order1Exponent5Hand = 5;
      assertEquals(order1Exponent5Func, order1Exponent5Hand, EPSILON);
      
      int order11Exponent1Func = septic.getDerivativeCoefficient(11, 1);
      int order11Exponent1Hand = 0;
      assertEquals(order11Exponent1Func, order11Exponent1Hand, EPSILON);
      
      int order13Exponent8Func = septic.getDerivativeCoefficient(13, 8);
      int order13Exponent8Hand = 0;
      assertEquals(order13Exponent8Func, order13Exponent8Hand, EPSILON);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDerivativeVersionsCubic()
   {
      //cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoPolynomial cubic = new YoPolynomial(namePrefix + "Cubic", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         double scaleDY0 = 1.0 / Math.random(), scaleDYf = 1.0 / Math.random();
         
         double x0 = Math.signum(Math.random()) * Math.random() * scaleX0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
         double dy0 = Math.signum(Math.random()) * Math.random() * scaleDY0, dyf = Math.signum(Math.random()) * Math.random() * scaleDYf;
               
         cubic.setCubic(x0, xf, y0, dy0, yf, dyf);
         
         double x = Math.random() * (xf - x0);
         
         compareDerivativeVersions(cubic, x);
      }
   }
   
   
   public void compareDerivativesPoint(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPoly = polynomial.getDerivative(i, x);
         
         double generalizedDYHand = 0.0;
         if(i < coefficients.length)
         {
            for(int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getDerivativeCoefficient(i, j);
               generalizedDYHand += coefficients[j] * derivativeCoefficient * Math.pow(x, j-i);
            }
         }
         else
         {
            generalizedDYHand = 0.0;
         }
         assertEquals(generalizedDYPoly, generalizedDYHand, EPSILON);
      }
   }
   
   public void compareXPowersDerivativesVector(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         DenseMatrix64F generalizedDYPoly = polynomial.getXPowersDerivativeVector(i, x);
         DenseMatrix64F generalizedDYHand = new DenseMatrix64F(generalizedDYPoly.getNumRows(), generalizedDYPoly.getNumCols());
         if(i < coefficients.length)
         {
            for(int j = i; j < coefficients.length; j++)
            {
               double derivativeCoefficient = polynomial.getDerivativeCoefficient(i, j);
               generalizedDYHand.set(j, 0, derivativeCoefficient * Math.pow(x, j - i));        
            }
         }
         for(int k = 0; k < coefficients.length; k++)
         {
            assertEquals(generalizedDYPoly.get(k, 0), generalizedDYHand.get(k, 0), EPSILON);
         }
      }
   }
   
   public void compareDerivativeVersions(YoPolynomial polynomial, double x)
   {
      double[] coefficients = polynomial.getCoefficients();
      for(int i = 0; i < coefficients.length + 3; i++)
      {
         double generalizedDYPolyScalar = polynomial.getDerivative(i, x);
         double generalizedDYHandScalar = 0.0;
         
         DenseMatrix64F generalizedDYPolyVector = polynomial.getXPowersDerivativeVector(i, x);
         for(int j = 0; j < generalizedDYPolyVector.numRows; j++)
         {
            generalizedDYHandScalar += generalizedDYPolyVector.get(j,0) * coefficients[j];
         }
         assertEquals(generalizedDYPolyScalar, generalizedDYHandScalar, EPSILON);
      }
   }
}