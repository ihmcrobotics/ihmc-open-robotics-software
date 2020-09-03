package us.ihmc.robotics.statistics;

import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class StandardDeviationCalculatorTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testNoVariance()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoDouble valueProvider = new YoDouble("valueProvider", testRegistry);
      OnlineStandardDeviationCalculator calculator = new OnlineStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      double value = 17.3;
      valueProvider.set(value);

      for (int i = 0; i < numberOfValues; i++)
      {
         calculator.update();
      }

      assertEquals(value, calculator.getMean(), epsilon);
      assertEquals(0.0, calculator.getStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getVariance(), epsilon);
   }

   @Test
   public void testFromList()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoDouble valueProvider = new YoDouble("valueProvider", testRegistry);
      OnlineStandardDeviationCalculator calculator = new OnlineStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Random random = new Random(1738L);
      TDoubleArrayList valueList = new TDoubleArrayList();
      double bias = 10.0;
      double seed = 5.0;

      for (int i = 0; i < numberOfValues; i++)
      {
         double value = bias + RandomNumbers.nextDouble(random, seed);
         valueList.add(value);
         valueProvider.set(value);

         calculator.update();
      }

      double summedValues = 0.0;
      for (int i = 0; i < numberOfValues; i++)
         summedValues += valueList.get(i);
      double mean = summedValues / numberOfValues;
      double sumOfSquares = 0.0;
      for (int i = 0; i < numberOfValues; i++)
         sumOfSquares += MathTools.square(valueList.get(i) - mean);
      double variance = sumOfSquares / numberOfValues;
      double populationVariance = sumOfSquares / (numberOfValues - 1);
      double standardDeviation = Math.sqrt(variance);

      assertEquals(mean, calculator.getMean(), epsilon);
      assertEquals(variance, calculator.getVariance(), epsilon);
      assertEquals(standardDeviation, calculator.getStandardDeviation(), epsilon);
   }
}
