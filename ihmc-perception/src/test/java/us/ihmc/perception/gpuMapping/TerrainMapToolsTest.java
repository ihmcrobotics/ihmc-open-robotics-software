package us.ihmc.perception.gpuMapping;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;

import static org.junit.jupiter.api.Assertions.*;

public class TerrainMapToolsTest
{
   private final static float MILLISECOND_TOLERANCE = 1.0f;
   private final int iterations = 1000;

   @Test
   public void testConvertToTerrainMapData()
   {
      double cellResolution = 0.2;
      double gridSizeXY = 1.0;

      TerrainMapData terrainMapData = new TerrainMapData(cellResolution, gridSizeXY, 0.0, 0.0);
      int cellsPerAxis = terrainMapData.getCellsPerAxis();

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(0));
      Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
      Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
      Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
      Mat traversabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));
      Mat traversabilityClassMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5));
      Mat collisionMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));

      TerrainMapTools.convertToTerrainMapData(heightMap,
                                              snapNormalXMap,
                                              snapNormalYMap,
                                              snapNormalZMap,
                                              traversabilityMap,
                                              traversabilityClassMap,
                                              collisionMap,
                                              new Point3D(0.0, 0.0, 0.0),
                                              terrainMapData);

      int totalCells = cellsPerAxis * cellsPerAxis;

      // Check results match the input
      byte[] snapNormalXMapResult = terrainMapData.getSnapNormalXMap();
      byte[] snapNormalYMapResult = terrainMapData.getSnapNormalYMap();
      byte[] snapNormalZMapResult = terrainMapData.getSnapNormalZMap();
      float[] traversabilityResult = terrainMapData.getTraversabilityScoreMap();
      byte[] traversabilityClassResult = terrainMapData.getTraversabilityClassMap();

      for (int i = 0; i < totalCells; i++)
      {
         int x = i % cellsPerAxis;
         int y = i / cellsPerAxis;

         assertEquals(snapNormalXMapResult[i], snapNormalXMap.ptr(x, y).get());
         assertEquals(snapNormalYMapResult[i], snapNormalYMap.ptr(x, y).get());
         assertEquals(snapNormalZMapResult[i], snapNormalZMap.ptr(x, y).get());
         assertEquals(traversabilityResult[i], traversabilityMap.ptr(x, y).getFloat());
         assertEquals(traversabilityClassResult[i], traversabilityClassMap.ptr(x, y).get());
      }
   }

   @Test
   public void testSpeedConvertToTerrainMapData()
   {
      double cellResolution = 0.2;
      double gridSizeXY = 1.0;

      TerrainMapData terrainMapData = new TerrainMapData(cellResolution, gridSizeXY, 0.0, 0.0);
      int cellsPerAxis = terrainMapData.getCellsPerAxis();

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(0));
      Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
      Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
      Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
      Mat traversabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));
      Mat traversabilityClassMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5));
      Mat collisionMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         TerrainMapTools.convertToTerrainMapData(heightMap,
                                                 snapNormalXMap,
                                                 snapNormalYMap,
                                                 snapNormalZMap,
                                                 traversabilityMap,
                                                 traversabilityClassMap,
                                                 collisionMap,
                                                 new Point3D(0.0, 0.0, 0.0),
                                                 terrainMapData);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Mats -> Terrain Map Data: %.6f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMat = MILLISECOND_TOLERANCE * 5;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
                            "Actual was: " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }
}
