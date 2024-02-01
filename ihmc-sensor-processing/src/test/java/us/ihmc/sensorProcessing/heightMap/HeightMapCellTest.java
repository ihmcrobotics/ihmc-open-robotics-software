package us.ihmc.sensorProcessing.heightMap;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class HeightMapCellTest
{
   @Test
   public void testConstantValueWithRandomVariance()
   {
      HeightMapParameters parameters = new HeightMapParameters();
      HeightMapCell cell = new HeightMapCell(parameters);
      Random random = new Random(1738);
      double height = 0.3;
      double kalmanFilterHeight = 0.3;
      double kalmanFilterVariance = MathTools.square(parameters.getNominalStandardDeviation());
      for (int i = 0; i < 100; i++)
      {
         double variance = RandomNumbers.nextDouble(random, 0.001, 0.5);
         cell.addPoint(height, variance);

         kalmanFilterHeight = (kalmanFilterHeight * variance + height * kalmanFilterVariance) / (variance + kalmanFilterVariance);
         kalmanFilterVariance = (variance * kalmanFilterVariance) / (variance + kalmanFilterVariance);
      }
      cell.updateHeightEstimate();

      assertEquals(height, cell.getEstimatedHeight(), 1e-5);
   }
}
