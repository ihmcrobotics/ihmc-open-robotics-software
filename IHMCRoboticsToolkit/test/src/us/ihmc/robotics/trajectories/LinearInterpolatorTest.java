package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class LinearInterpolatorTest
{
   private double x0 = 0.0;
   private double y0 = 0.0;

   private double x1 = 1.0;
   private double y1 = 1.0;

   private double slope = (y1 - y0) / (x1 - x0);
   private double yIntercept = y0 - slope * x0;


   private double xTest = 0.5;
   private double yTest = slope * xTest + yIntercept;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolationSimple()
   {
      double xPointValue, yPointValue, yPointValueExpected;

      int numberOfPoints = 2;
      double[] xPoints = new double[numberOfPoints];
      double[] yPoints = new double[numberOfPoints];

      xPoints[0] = x0;
      yPoints[0] = y0;

      xPoints[1] = x1;
      yPoints[1] = y1;

      LinearInterpolater linearInterpolater = new LinearInterpolater(xPoints, yPoints);

      xPointValue = 0.5;

      yPointValueExpected = slope * xPointValue + yIntercept;
      yPointValue = linearInterpolater.getPoint(xPointValue);


      assertEquals(yPointValueExpected, yPointValue, 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolationSimpleWithArrayListConstructor()
   {
      double xPointValue, yPointValue, yPointValueExpected;

      int numberOfPoints = 2;
      ArrayList<Double> xPointsArrayList = new ArrayList<Double>(numberOfPoints);
      ArrayList<Double> yPointsArrayList = new ArrayList<Double>(numberOfPoints);

      xPointsArrayList.add(x0);
      yPointsArrayList.add(y0);

      xPointsArrayList.add(x1);
      yPointsArrayList.add(y1);

      LinearInterpolater linearInterpolater;
      try
      {
         linearInterpolater = new LinearInterpolater(xPointsArrayList, yPointsArrayList);

         xPointValue = 0.5;

         yPointValueExpected = slope * xPointValue + yIntercept;
         yPointValue = linearInterpolater.getPoint(xPointValue);


         assertEquals(yPointValueExpected, yPointValue, 1e-5);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolationSimpleWithInteroplatedIndexConstructor()
   {
      double xPointValue, yPointValue, yPointValueExpected;

      int numberOfPoints = 2;
      ArrayList<Double> xPointsArrayList = new ArrayList<Double>(numberOfPoints);

      xPointsArrayList.add(x0);
      xPointsArrayList.add(x1);

      LinearInterpolater linearInterpolater;
      try
      {
         linearInterpolater = new LinearInterpolater(xPointsArrayList);

         xPointValue = x0;

         yPointValueExpected = 0.0;
         yPointValue = linearInterpolater.getPoint(xPointValue);

         assertEquals(yPointValueExpected, yPointValue, 1e-5);


         xPointValue = x1;

         yPointValueExpected = 1.0;
         yPointValue = linearInterpolater.getPoint(xPointValue);

         assertEquals(yPointValueExpected, yPointValue, 1e-5);


         double indexFraction = 0.73;
         xPointValue = x1 * indexFraction;

         yPointValueExpected = indexFraction;
         yPointValue = linearInterpolater.getPoint(xPointValue);

         assertEquals(yPointValueExpected, yPointValue, 1e-5);

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolationBetweenTwoRandomPoints()
   {
      double xPointValue, yPointValue, yPointValueExpected, xIncrement;

      int numberOfPoints = 2;
      double[] xPoints = new double[numberOfPoints];
      double[] yPoints = new double[numberOfPoints];

      Random random = new Random(100L);
      double scale = 10.0;

      int numberOfTests = 100;
      int numberOfEvaluationsPerTest = 10;

      for (int i = 0; i < numberOfTests; i++)
      {
         double slope = scale * (random.nextDouble() - 0.5);
         double yIntercept = scale * (random.nextDouble() - 0.5);

         xPoints[0] = scale * (random.nextDouble() - 0.5);
         yPoints[0] = slope * xPoints[0] + yIntercept;

         xPoints[1] = xPoints[0] + Math.abs(scale * random.nextDouble());
         yPoints[1] = slope * xPoints[1] + yIntercept;

         LinearInterpolater linearInterpolater = new LinearInterpolater(xPoints, yPoints);

         xIncrement = (xPoints[1] - xPoints[0]) / numberOfEvaluationsPerTest;

         for (int j = 0; j <= numberOfEvaluationsPerTest; j++)
         {
            xPointValue = xPoints[0] + j * xIncrement;
            yPointValueExpected = slope * xPointValue + yIntercept;
            yPointValue = linearInterpolater.getPoint(xPointValue);


            assertEquals(yPointValueExpected, yPointValue, 1e-5);
         }
      }
   }
}
