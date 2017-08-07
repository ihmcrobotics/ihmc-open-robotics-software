package us.ihmc.robotics.functionApproximation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class LinearRegressionTest
{
   private static final boolean VERBOSE = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTypicalExampleOne()
   {
      Random random = new Random(1984L);
      ArrayList<double[]> inputs = new ArrayList<double[]>();
      ArrayList<Double> outputs = new ArrayList<Double>();

      double xCoefficient = 1.0;
      double xSquaredCoefficient = 5.0;

      for (int i = 0; i < 500; i++)
      {
         double x = (random.nextDouble() * 2.0) - xCoefficient;
         double[] input = new double[] {xCoefficient, x, x * x};

         double output = (random.nextDouble() * 0.1) + xCoefficient * x + xSquaredCoefficient * x * x;
         inputs.add(input);
         outputs.add(output);
      }

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      double runTimeInMilliseconds = solveAndReturnRuntimeInMilliseconds(linearRegression);

      double[] coefficientArray = new double[3];
      linearRegression.getCoefficientVector(coefficientArray);
      printResults(linearRegression, runTimeInMilliseconds, coefficientArray);

      double epsilon = 0.1;
      double maxSquaredError = 1e-3;
      double maxRuntimeInMilliseconds = 7.2;
      double[] expectedCoefficients = new double[] {0.05, xCoefficient, xSquaredCoefficient};

      assertResultsAreAsExpected(linearRegression, runTimeInMilliseconds, coefficientArray, epsilon, maxSquaredError, maxRuntimeInMilliseconds,
                                 expectedCoefficients);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTypicalExampleTwo()
   {
      Random random = new Random(1776L);
      int numberOfPoints = 500;

      double[][] inputs = new double[numberOfPoints][];
      double[] outputs = new double[numberOfPoints];

      double xCoefficient = 1.0;
      double xSquaredCoefficient = 0.0;
      double yCoefficient = 0.2;
      double ySquaredCoefficient = -3.0;
      double xyCoefficient = 5.0;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = (random.nextDouble() * 2.0) - 1.0;
         double y = (random.nextDouble() * 2.0) - 1.0;
         double[] input = new double[]
         {
            1.0, x, x * x, y, y * y, x * y
         };
         double output = 4.0 + (random.nextDouble() * 0.1) + xCoefficient * x + xSquaredCoefficient * x * x + yCoefficient * y + ySquaredCoefficient * y * y
                         + xyCoefficient * x * y;

         inputs[i] = input;
         outputs[i] = output;
      }

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      double runTimeInMilliseconds = solveAndReturnRuntimeInMilliseconds(linearRegression);

      double[] coefficientArray = new double[6];
      linearRegression.getCoefficientVector(coefficientArray);
      printResults(linearRegression, runTimeInMilliseconds, coefficientArray);

      double epsilon = 0.1;
      double maxSquaredError = 1e-3;
      double maxRuntimeInMilliseconds = 5.0;
      double[] expectedCoefficients = new double[]
      {
         4.05, xCoefficient, xSquaredCoefficient, yCoefficient, ySquaredCoefficient, xyCoefficient
      };

      assertResultsAreAsExpected(linearRegression, runTimeInMilliseconds, coefficientArray, epsilon, maxSquaredError, maxRuntimeInMilliseconds,
                                 expectedCoefficients);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPerfectMatch()
   {
      Random random = new Random(2000L);
      ArrayList<double[]> inputs = new ArrayList<double[]>();
      ArrayList<Double> outputs = new ArrayList<Double>();

      double unitsCoefficient = 90.0;
      double xCoefficient = 5.0;
      double xSquaredCoefficient = 6.0;
      double yCoefficient = 7.0;
      double ySquaredCoefficient = -8.0;
      double xyCoefficient = 13.0;

      for (int i = 0; i < 500; i++)
      {
         double x = (random.nextDouble() * 4.0) - 1.0;
         double y = (random.nextDouble() * 4.0) - 1.0;
         double[] input = new double[]
         {
            1.0, x, x * x, y, y * y, x * y
         };
         double output = unitsCoefficient + xCoefficient * x + xSquaredCoefficient * x * x + yCoefficient * y + ySquaredCoefficient * y * y
                         + xyCoefficient * x * y;
         inputs.add(input);
         outputs.add(output);
      }

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      double runTimeInMilliseconds = solveAndReturnRuntimeInMilliseconds(linearRegression);

      double[] coefficientArray = new double[6];
      linearRegression.getCoefficientVector(coefficientArray);
      printResults(linearRegression, runTimeInMilliseconds, coefficientArray);

      double epsilon = 1e-10;
      double maxSquaredError = 1e-14;
      double maxRuntimeInMilliseconds = 5.0;
      double[] expectedCoefficients = new double[]
      {
         unitsCoefficient, xCoefficient, xSquaredCoefficient, yCoefficient, ySquaredCoefficient, xyCoefficient
      };

      assertResultsAreAsExpected(linearRegression, runTimeInMilliseconds, coefficientArray, epsilon, maxSquaredError, maxRuntimeInMilliseconds,
                                 expectedCoefficients);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomness()
   {
      Random random = new Random(1776L);
      ArrayList<double[]> inputs = new ArrayList<double[]>();
      ArrayList<Double> outputs = new ArrayList<Double>();

      for (int i = 0; i < 5; i++)
      {
         double[] input = new double[] {random.nextDouble() * 100.0, random.nextDouble() * 10.0};
         double output = random.nextDouble() * 500.0;

         inputs.add(input);
         outputs.add(output);
      }

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      boolean foundSolution = linearRegression.solve();
      assertTrue(foundSolution);    // It's even finding a solution here. Should it really? Where would it not find one then?

      double squaredError = linearRegression.getSquaredError();
      if (VERBOSE)
      {
         System.out.println("\n squaredError = " + squaredError);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testNotEnoughPoints()
   {
      ArrayList<double[]> inputs = new ArrayList<double[]>();
      ArrayList<Double> outputs = new ArrayList<Double>();

      inputs.add(new double[]{1.0});
      outputs.add(2.0);
      outputs.add(3.0);

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      boolean foundSolution = linearRegression.solve();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testAskingForAnswerBeforeDone()
   {
      double[][] inputs = new double[][]
      {
         {1.0}, {1.0}
      };
      double[] outputs = new double[] {1.0, 1.0};
      double[] coefficientVector = new double[1];

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      linearRegression.getCoefficientVector(coefficientVector);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testAskingForSquaredErrorBeforeDone()
   {
      double[][] inputs = new double[][]
      {
         {1.0}, {1.0}
      };
      double[] outputs = new double[] {1.0, 1.0};

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      linearRegression.getSquaredError();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testAskingForCoefficientVectorAsMatrixBeforeDone()
   {
      double[][] inputs = new double[][]
      {
         {1.0}, {1.0}
      };
      double[] outputs = new double[] {1.0, 1.0};

      LinearRegression linearRegression = new LinearRegression(inputs, outputs);
      linearRegression.getCoefficientVectorAsMatrix();
   }


   private void assertResultsAreAsExpected(LinearRegression linearRegression, double runTimeInMilliseconds, double[] coefficientArray, double epsilon,
           double maxSquaredError, double maxRuntimeInMilliseconds, double[] expectedCoefficients)
   {
      assertEquals(expectedCoefficients.length, coefficientArray.length);

      for (int i = 0; i < expectedCoefficients.length; i++)
      {
         assertEquals(expectedCoefficients[i], coefficientArray[i], epsilon);

      }

      assertTrue("linearRegression.getSquaredError() was less than 0.0!", linearRegression.getSquaredError() > 0.0);
      assertTrue("linearRegression.getSquaredError() = " + linearRegression.getSquaredError(), linearRegression.getSquaredError() < maxSquaredError);

//      assertTrue("Run time of " + runTimeInMilliseconds + " msec was too slow. Trying to hit " + maxRuntimeInMilliseconds, runTimeInMilliseconds < maxRuntimeInMilliseconds);
   }



   private void printResults(LinearRegression linearRegression, double runTimeInMilliseconds, double[] coefficientArray)
   {
      if (VERBOSE)
      {
         System.out.println("\nLinearRegressionTest:: regression took " + runTimeInMilliseconds + " ms");

         System.out.print("LinearRegressionTest:: coefficients are (");

         for (int i = 0; i < coefficientArray.length - 1; i++)
         {
            System.out.print(coefficientArray[i] + ", ");
         }

         System.out.println(coefficientArray[coefficientArray.length - 1] + ")");

         System.out.println("LinearRegressionTest:: linearRegression.getSquaredError() : " + linearRegression.getSquaredError());
      }
   }

   private double solveAndReturnRuntimeInMilliseconds(LinearRegression linearRegression)
   {
      long startTime = System.nanoTime();
      boolean solveSucceeded = linearRegression.solve();
      assertTrue(solveSucceeded);
      long endTime = System.nanoTime();
      double runTimeInMilliseconds = ((double) (endTime - startTime)) / 1000000.0;

      return runTimeInMilliseconds;
   }

}
