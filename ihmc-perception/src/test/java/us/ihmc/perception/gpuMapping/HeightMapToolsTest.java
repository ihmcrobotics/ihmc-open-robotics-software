package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapToolsTest
{
   private final static float MILLISECOND_TOLERANCE = 1.0f;
   private final int iterations = 1000;
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

   @Test
   public void testComputeTransformSVDTranslationOnly()
   {
      float[] localData = new float[] {
            0f, 0f, 0f, 1f,
            1f, 0f, 0f, 1f,
            0f, 1f, 0f, 1f
      };

      float[] globalData = new float[] {
            1f, 2f, 3f, 1f,
            2f, 2f, 3f, 1f,
            1f, 3f, 3f, 1f
      };

      int[] corr = new int[] {0, 1, 2};
      int numPoints = 3;

      DMatrixRMaj transform = HeightMapTools.computeTransformSVD(localData, globalData, corr, numPoints);

      // Translation should be (1,2,3)
      assertEquals(1.0, transform.get(0, 3), 1e-6);
      assertEquals(2.0, transform.get(1, 3), 1e-6);
      assertEquals(3.0, transform.get(2, 3), 1e-6);

      // Rotation should be identity
      assertEquals(1.0, transform.get(0, 0), 1e-6);
      assertEquals(1.0, transform.get(1, 1), 1e-6);
      assertEquals(1.0, transform.get(2, 2), 1e-6);

      assertEquals(0.0, transform.get(0, 1), 1e-6);
      assertEquals(0.0, transform.get(0, 2), 1e-6);
      assertEquals(0.0, transform.get(1, 0), 1e-6);
      assertEquals(0.0, transform.get(1, 2), 1e-6);
      assertEquals(0.0, transform.get(2, 0), 1e-6);
      assertEquals(0.0, transform.get(2, 1), 1e-6);
   }

   @Test
   public void testComputeTransformSVDRotationAndTranslation()
   {
      // 3 points forming a triangle in local frame
      float[] localData = new float[] {
            0f, 0f, 0f, 1f,
            1f, 0f, 0f, 1f,
            0f, 1f, 0f, 1f
      };

      // Apply 90° rotation around Z-axis + translation (1, 2, 3)
      // Rotation 90° CCW around Z: x' = -y, y' = x, z' = z
      float[] globalData = new float[] {
            1f, 2f, 3f, 1f,   // (0,0,0) + translation
            1f, 3f, 3f, 1f,   // (1,0,0) rotated 90° + translation
            0f, 2f, 3f, 1f    // (0,1,0) rotated 90° + translation
      };

      // Correspondences are 0->0, 1->1, 2->2
      int[] corr = new int[] {0, 1, 2};
      int numPoints = 3;

      DMatrixRMaj transform = HeightMapTools.computeTransformSVD(localData, globalData, corr, numPoints);

      // Translation should be (1,2,3)
      assertEquals(1.0, transform.get(0, 3), 1e-6);
      assertEquals(2.0, transform.get(1, 3), 1e-6);
      assertEquals(3.0, transform.get(2, 3), 1e-6);

      // Rotation should be 90° around Z
      assertEquals(0.0, transform.get(0, 0), 1e-6);
      assertEquals(-1.0, transform.get(0, 1), 1e-6);
      assertEquals(0.0, transform.get(0, 2), 1e-6);

      assertEquals(1.0, transform.get(1, 0), 1e-6);
      assertEquals(0.0, transform.get(1, 1), 1e-6);
      assertEquals(0.0, transform.get(1, 2), 1e-6);

      assertEquals(0.0, transform.get(2, 0), 1e-6);
      assertEquals(0.0, transform.get(2, 1), 1e-6);
      assertEquals(1.0, transform.get(2, 2), 1e-6);
   }

   @Test
   public void testReflectionIsCorrected()
   {
      // Local points: simple right triangle
      float[] localData = new float[] {
            0f, 0f, 0f, 1f,
            1f, 0f, 0f, 1f,
            0f, 1f, 0f, 1f
      };

      // Global points: mirrored across Y axis + translated
      float[] globalData = new float[] {
            1f, 2f, 3f, 1f,  // mirrored + translation
            0f, 2f, 3f, 1f,
            1f, 3f, 3f, 1f
      };

      // Identity correspondences
      int[] corr = new int[] {0, 1, 2};
      int numberOfPoints = 3;

      DMatrixRMaj T = HeightMapTools.computeTransformSVD(localData, globalData, corr, numberOfPoints);

      DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.extract(T, 0, 3, 0, 3, rotation, 0, 0);

      // Ensure it is a proper rotation (det > 0)
      assertTrue(CommonOps_DDRM.det(rotation) > 0.0);
   }

   @Test
   public void testComputeTransformSVDWithMoreThanThreePoints()
   {
      // Local points (forming a square + diagonal)
      float[] localData = new float[] {
            0f, 0f, 0f, 1f,
            1f, 0f, 0f, 1f,
            0f, 1f, 0f, 1f,
            1f, 1f, 0f, 1f,
            0.5f, 0.5f, 0f, 1f
      };

      // Apply rotation 90° around Z + translation (1,2,3)
      // x' = -y, y' = x, z' = z
      float[] globalData = new float[] {
            1f, 2f, 3f, 1f,    // rotated + translated
            1f, 3f, 3f, 1f,
            0f, 2f, 3f, 1f,
            0f, 3f, 3f, 1f,
            0.5f, 2.5f, 3f, 1f
      };

      // Identity correspondences
      int[] corr = new int[] {0, 1, 2, 3, 4};
      int numberOfPoints = 5;

      DMatrixRMaj T = HeightMapTools.computeTransformSVD(localData, globalData, corr, numberOfPoints);

      DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.extract(T, 0, 3, 0, 3, rotation, 0, 0);

      double tx = T.get(0, 3);
      double ty = T.get(1, 3);
      double tz = T.get(2, 3);

      assertEquals(1.0, tx, 1e-6);
      assertEquals(2.0, ty, 1e-6);
      assertEquals(3.0, tz, 1e-6);

      // Rotation should be 90° around Z
      assertEquals(0.0, rotation.get(0, 0), 1e-6);
      assertEquals(-1.0, rotation.get(0, 1), 1e-6);
      assertEquals(0.0, rotation.get(0, 2), 1e-6);

      assertEquals(1.0, rotation.get(1, 0), 1e-6);
      assertEquals(0.0, rotation.get(1, 1), 1e-6);
      assertEquals(0.0, rotation.get(1, 2), 1e-6);

      assertEquals(0.0, rotation.get(2, 0), 1e-6);
      assertEquals(0.0, rotation.get(2, 1), 1e-6);
      assertEquals(1.0, rotation.get(2, 2), 1e-6);

      // Rotation determinant should be positive
      assertTrue(CommonOps_DDRM.det(rotation) > 0.0);
   }
}
