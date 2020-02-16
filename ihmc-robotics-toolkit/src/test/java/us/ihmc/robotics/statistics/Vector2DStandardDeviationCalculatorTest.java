package us.ihmc.robotics.statistics;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class Vector2DStandardDeviationCalculatorTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testNoVariance()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry(getClass().getSimpleName());
      YoFrameVector2D valueProvider = new YoFrameVector2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Vector2DStandardDeviationCalculator calculator = new Vector2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D value = new Vector2D(17.3, 5.6);
      valueProvider.set(value);

      for (int i = 0; i < numberOfValues; i++)
      {
         calculator.update();
      }

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(value, calculator.getMean(), epsilon);
      assertEquals(0.0, calculator.getStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getVariance(), epsilon);
   }

   @Test
   public void testFromList()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry(getClass().getSimpleName());
      YoFrameVector2D valueProvider = new YoFrameVector2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Vector2DStandardDeviationCalculator calculator = new Vector2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Random random = new Random(1738L);
      List<Vector2D> valueList = new ArrayList<>();
      double bias = 10.0;
      double seed = 5.0;

      for (int i = 0; i < numberOfValues; i++)
      {
         Vector2D value = new Vector2D(bias, bias);
         value.add(RandomNumbers.nextDouble(random, seed), RandomNumbers.nextDouble(random, seed));
         valueList.add(value);
         valueProvider.set(value);

         calculator.update();
      }

      Vector2D summedValues = new Vector2D();
      for (int i = 0; i < numberOfValues; i++)
         summedValues.add(valueList.get(i));

      Vector2D mean = new Vector2D(summedValues);
      mean.scale(1.0 / numberOfValues);

      double sumOfSquares = 0.0;
      for (int i = 0; i < numberOfValues; i++)
         sumOfSquares += MathTools.square(valueList.get(i).dot(mean) - mean.length());
      double variance = sumOfSquares / numberOfValues;
      double populationVariance = sumOfSquares / (numberOfValues - 1);
      double standardDeviation = Math.sqrt(variance);

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(mean, calculator.getMean(), epsilon);
      assertEquals(variance, calculator.getVariance(), epsilon);
      assertEquals(standardDeviation, calculator.getStandardDeviation(), epsilon);
   }
}
