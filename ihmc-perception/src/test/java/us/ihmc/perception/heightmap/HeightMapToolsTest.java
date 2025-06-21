package us.ihmc.perception.heightmap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.util.Random;

public class HeightMapToolsTest
{
   private static final boolean DEBUGGING = false;
   private final int iterations = 1000;
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();

   @Test
   public void testSpeedConvertHeightMapDataToMat()
   {
      int cellsPerAxis = 500;
      double gridSizeXY = cellsPerAxis * 0.02;

      HeightMapData heightMapData = new HeightMapData(0.02, gridSizeXY, 0.0, 0.0);
      int cellsPerAxisAdjustedForHeightMapData = cellsPerAxis + 1;
      for (int i = 0; i < cellsPerAxisAdjustedForHeightMapData; i++)
      {
         for (int j = 0; j < cellsPerAxisAdjustedForHeightMapData; j++)
         {
            int key = j * cellsPerAxisAdjustedForHeightMapData + i;
            heightMapData.setHeightAt(key, 1.0f);
         }
      }

      if (DEBUGGING)
         PerceptionDebugTools.printMat("Input Mat (HeightMapData)", heightMapData.getHeightMat(), 1);

      Mat heightMap = new Mat(heightMapData.getCellsPerAxis(), heightMapData.getCellsPerAxis(), heightMapData.getHeightMat().type());
      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapTools.convertHeightMapDataToMat(heightMap, heightMapData);
      }

      if (DEBUGGING)
         PerceptionDebugTools.printMat("Result", heightMap, 1);

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack: %.3f ms%n", averageTimePerIteration);
   }

   @Test
   public void testSpeedConvertMatToHeightMapData()
   {
      int cellsPerAxis = 500;
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMapMat.ptr(i, j).putFloat(1.0f);
         }
      }

      HeightMapData heightMapData = new HeightMapData(0.02, 10.0, 0.0, 0.0);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapTools.convertToHeightMapData(heightMapMat,
                                               heightMapData,
                                               new Point3D(0.0, 0.0, 0.0),
                                               (float) heightMapParameters.getTerrainWidthInMeters(),
                                               (float) heightMapParameters.getCellSizeInMeters(),
                                               heightMapParameters);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack: %.3f ms%n", averageTimePerIteration);
   }

   @Test
   public void testIndexing()
   {
      Random random = new Random(328923);
      int iterations = 1000;

      for (int i = 0; i < iterations; i++)
      {
         double resolution = EuclidCoreRandomTools.nextDouble(random, 1e-4, 0.5);
         double gridCenterX = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double gridCenterY = EuclidCoreRandomTools.nextDouble(random, 1.0);

         int centerIndex = 1 + random.nextInt(50);
         double gridSizeXY = 2.0 * centerIndex * resolution;

         Assertions.assertEquals(centerIndex, HeightMapTools.computeCenterIndex(gridSizeXY, resolution), "Invalid cell per axis calculation");

         double xCoordinate = gridCenterX + EuclidCoreRandomTools.nextDouble(random, 0.5 * gridSizeXY);
         double yCoordinate = gridCenterY + EuclidCoreRandomTools.nextDouble(random, 0.5 * gridSizeXY);

         int key = HeightMapTools.coordinateToKey(xCoordinate, yCoordinate, gridCenterX, gridCenterY, resolution, centerIndex);
         double xCoordinateOnGrid = HeightMapTools.keyToXCoordinate(key, gridCenterX, resolution, centerIndex);
         double yCoordinateOnGrid = HeightMapTools.keyToYCoordinate(key, gridCenterY, resolution, centerIndex);

         Assertions.assertTrue(Math.abs(xCoordinate - xCoordinateOnGrid) < 0.5 * resolution + 1e-10, "Invalid key-coordinate conversion");
         Assertions.assertTrue(Math.abs(yCoordinate - yCoordinateOnGrid) < 0.5 * resolution + 1e-10, "Invalid key-coordinate conversion");
      }
   }
}
