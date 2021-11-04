package us.ihmc.sensorProcessing.heightMap;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

public class HeightMapToolsTest
{
   @Test
   public void testIndexing()
   {
      Random random = new Random(328923);
      int iterations = 200;

      for (int i = 0; i < iterations; i++)
      {
         double resolution = EuclidCoreRandomTools.nextDouble(random, 1e-4, 0.5);
         double gridCenter = EuclidCoreRandomTools.nextDouble(random, 1.0);
         int minMaxIndex = 1 + random.nextInt(50);
         double gridSizeXY = 2.0 * minMaxIndex * resolution;
         int computedMinMaxCoordinate = HeightMapTools.minMaxIndex(gridSizeXY, resolution);
         Assertions.assertEquals(minMaxIndex, computedMinMaxCoordinate, "Min max coordinates in height map are not equal");

         double coordinate = EuclidCoreRandomTools.nextDouble(random, gridSizeXY);
         int index = HeightMapTools.toIndex(coordinate, gridCenter, resolution, minMaxIndex);
         double roundedCoordinate = HeightMapTools.toCoordinate(index, gridCenter, resolution, minMaxIndex);
         int recomputedIndex = HeightMapTools.toIndex(roundedCoordinate, gridCenter, resolution, minMaxIndex);
         Assertions.assertEquals(index, recomputedIndex, "Height map indexing computes incorrect coordinate");
         Assertions.assertTrue(Math.abs(coordinate - roundedCoordinate) < 0.5 * resolution + 1e-10, "Height map indexing computes incorrect coordinate");
      }
   }
}
