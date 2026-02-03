package us.ihmc.perception.gpuMapping;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapToolsTest
{
   private final int iterations = 1000;
   private final static float MILLISECOND_TOLERANCE = 1.0f;
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();

   /**
    * Dealing with row-major or column-major is confusing.
    * This test doesn't really test anything, but we want to show that we can work with both row major and column major data.
    * Because the method for converting and unconverting the height map data is the same, it doesn't matter if our input mat is row or column-major.
    */
   @Test
   public void testRowMajor()
   {
      // Setup default values
      double cellSize = 0.2;
      double terrainWidth = 1.2;
      double centerX = 0.0;
      double centerY = 0.0;
      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;
      Point3D centerLocation = new Point3D(centerX, centerY, 0.0);

      // ROW-MAJOR INPUT
      {
         Mat originalMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);

         for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
         {
            // Insert into array in row-major
            int x = i % cellsPerAxis;
            int y = i / cellsPerAxis;

            originalMat.ptr(y, x).putFloat(i);
         }

         HeightMapData heightMapData = new HeightMapData(cellSize, terrainWidth, centerX, centerY);
         HeightMapTools.convertToHeightMapData(originalMat, heightMapData, centerLocation, (float) terrainWidth, (float) cellSize);

         Mat newData = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
         HeightMapTools.convertHeightMapDataToMat(newData, heightMapData);

         for (int x = 0; x < cellsPerAxis; x++)
         {
            for (int y = 0; y < cellsPerAxis; y++)
            {
               assertEquals(originalMat.ptr(x, y).getFloat(), newData.ptr(x, y).getFloat());
            }
         }
      }

      // COLUMN-MAJOR INPUT
      {
         Mat originalMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
         for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
         {
            // Insert into array in column-major
            int x = i / cellsPerAxis;
            int y = i % cellsPerAxis;

            originalMat.ptr(x, y).putFloat(i);
         }

         HeightMapData heightMapData = new HeightMapData(cellSize, terrainWidth, centerX, centerY);
         HeightMapTools.convertToHeightMapData(originalMat, heightMapData, centerLocation, (float) terrainWidth, (float) cellSize);

         Mat newData = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
         HeightMapTools.convertHeightMapDataToMat(newData, heightMapData);

         for (int x = 0; x < cellsPerAxis; x++)
         {
            for (int y = 0; y < cellsPerAxis; y++)
            {
               assertEquals(originalMat.ptr(x, y).getFloat(), newData.ptr(x, y).getFloat());
            }
         }
      }
   }

   @Test
   public void testRoundTripConvertMatToHeightMapDataAndBack()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();

      double cellSize = heightMapParameters.getCellSize();
      double terrainWidth = heightMapParameters.getGlobalWidthInMeters();
      double centerX = 0.0;
      double centerY = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;
      Point3D centerLocation = new Point3D(centerX, centerY, 0.0);

      Mat originalMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(1000));
      HeightMapData heightMapData = new HeightMapData(cellSize, terrainWidth, centerX, centerY);
      HeightMapTools.convertToHeightMapData(originalMat, heightMapData, centerLocation, (float) terrainWidth, (float) cellSize);

      Mat newData = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      HeightMapTools.convertHeightMapDataToMat(newData, heightMapData);

      for (int x = 0; x < cellsPerAxis; x++)
      {
         for (int y = 0; y < cellsPerAxis; y++)
         {
            assertEquals(originalMat.ptr(x, y).getFloat(), newData.ptr(x, y).getFloat());
         }
      }
   }

   @Test
   public void testSpeedConvertHeightMapDataToMat()
   {
      int cellsPerAxis = 500;
      HeightMapData heightMapData = new HeightMapData(0.02, 10.0, 0.0, 0.0);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMapData.setHeight(i, 1.0f);
         }
      }

      Mat heightMap = new Mat(heightMapData.getCellsPerAxis(), heightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapTools.convertHeightMapDataToMat(heightMap, heightMapData);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack [ HeightMapData -> Mat ]: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackHeightMapMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMatInMillis);
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
                                               (float) heightMapParameters.getGlobalWidthInMeters(),
                                               (float) heightMapParameters.getCellSize());
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack [ Mat -> HeightMapData ]: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackHeightMapMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMatInMillis);
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

         assertEquals(centerIndex, HeightMapTools.computeCenterIndex(gridSizeXY, resolution), "Invalid cell per axis calculation");

         double xCoordinate = gridCenterX + EuclidCoreRandomTools.nextDouble(random, 0.5 * gridSizeXY);
         double yCoordinate = gridCenterY + EuclidCoreRandomTools.nextDouble(random, 0.5 * gridSizeXY);

         int keyFromCoordinates = HeightMapTools.coordinateToKey(xCoordinate, yCoordinate, gridCenterX, gridCenterY, resolution, centerIndex);
         double xCoordinateOnGrid = HeightMapTools.keyToXCoordinate(keyFromCoordinates, gridCenterX, resolution, centerIndex);
         double yCoordinateOnGrid = HeightMapTools.keyToYCoordinate(keyFromCoordinates, gridCenterY, resolution, centerIndex);

         Assertions.assertTrue(Math.abs(xCoordinate - xCoordinateOnGrid) < 0.5 * resolution + 1e-10, "Invalid key-coordinate conversion");
         Assertions.assertTrue(Math.abs(yCoordinate - yCoordinateOnGrid) < 0.5 * resolution + 1e-10, "Invalid key-coordinate conversion");

         int xIndex = HeightMapTools.coordinateToIndex(xCoordinateOnGrid, gridCenterX, resolution, centerIndex);
         int yIndex = HeightMapTools.coordinateToIndex(yCoordinateOnGrid, gridCenterY, resolution, centerIndex);
         int keyFromIndices = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
         Assertions.assertEquals(keyFromCoordinates, keyFromIndices);
      }
   }
}
