package us.ihmc.robotics.statistics;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class Point2DStandardDeviationCalculatorTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testNoVariance()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoFramePoint2D valueProvider = new YoFramePoint2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Point2DStandardDeviationCalculator calculator = new Point2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Point2D value = new Point2D(17.3, 5.6);
      valueProvider.set(value);

      for (int i = 0; i < numberOfValues; i++)
      {
         calculator.update();
      }

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(value, calculator.getMean(), epsilon);
      assertEquals(0.0, calculator.getStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getVariance(), epsilon);
   }

   @Test
   public void testFromList()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoFramePoint2D valueProvider = new YoFramePoint2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Point2DStandardDeviationCalculator calculator = new Point2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Random random = new Random(1738L);
      List<Point2D> valueList = new ArrayList<>();
      double bias = 10.0;
      double seed = 5.0;

      for (int i = 0; i < numberOfValues; i++)
      {
         Point2D value = new Point2D(bias, bias);
         value.add(RandomNumbers.nextDouble(random, seed), RandomNumbers.nextDouble(random, seed));
         valueList.add(value);
         valueProvider.set(value);

         calculator.update();
      }

      Point2D summedValues = new Point2D();
      for (int i = 0; i < numberOfValues; i++)
         summedValues.add(valueList.get(i));

      Point2D mean = new Point2D(summedValues);
      mean.scale(1.0 / numberOfValues);

      double sumOfSquares = 0.0;
      for (int i = 0; i < numberOfValues; i++)
         sumOfSquares += valueList.get(i).distanceSquared(mean);
      double variance = sumOfSquares / numberOfValues;
      double populationVariance = sumOfSquares / (numberOfValues - 1);
      double standardDeviation = Math.sqrt(variance);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(mean, calculator.getMean(), epsilon);
      assertEquals(variance, calculator.getVariance(), epsilon);
      assertEquals(standardDeviation, calculator.getStandardDeviation(), epsilon);
   }
}
