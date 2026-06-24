package us.ihmc.perception.gpuMapping;

import static org.junit.jupiter.api.Assertions.*;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import perception_msgs.TerrainMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;

public class TerrainMapMessageToolsTest
{
   private final int iterations = 1000;
   private final static float MILLISECOND_TOLERANCE = 1.0f;

   @Test
   public void testMessagingRoundTrip()
   {
      double cellResolution = 0.2;
      double gridSizeXY = 1.0;
      int cellsPerAxis = (int) (gridSizeXY / cellResolution);

      TerrainMapData terrainMapData = new TerrainMapData(cellResolution, gridSizeXY, 0.0, 0.0);

      try (Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(0));
           Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
           Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
           Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
           Mat traversabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));
           Mat traversabilityClassMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5)))
      {
         TerrainMapTools.convertToTerrainMapData(heightMap,
                                                 snapNormalXMap,
                                                 snapNormalYMap,
                                                 snapNormalZMap,
                                                 traversabilityMap,
                                                 traversabilityClassMap,
                                                 new Point3D(0.0, 0.0, 0.0),
                                                 terrainMapData);

         TerrainMapMessage message = new TerrainMapMessage();
         TerrainMapMessageTools.toMessage(terrainMapData, message);

         TerrainMapData resultingData = TerrainMapMessageTools.unpackMessage(message);

         // Check results match the input
         byte[] snapNormalXMapResult = resultingData.getSnapNormalXMap();
         byte[] snapNormalYMapResult = resultingData.getSnapNormalYMap();
         byte[] snapNormalZMapResult = resultingData.getSnapNormalZMap();
         float[] traversabilityMapResult = resultingData.getTraversabilityScoreMap();
         byte[] traversabilityClassMapResult = resultingData.getTraversabilityClassMap();

         int totalCells = cellsPerAxis * cellsPerAxis;

         for (int i = 0; i < totalCells; i++)
         {
            int x = i % cellsPerAxis;
            int y = i / cellsPerAxis;

            assertEquals(snapNormalXMapResult[i], snapNormalXMap.ptr(x).get());
            assertEquals(snapNormalYMapResult[i], snapNormalYMap.ptr(x).get());
            assertEquals(snapNormalZMapResult[i], snapNormalZMap.ptr(x).get());
            assertEquals(traversabilityMapResult[i], traversabilityMap.ptr(x).getFloat());
            assertEquals(traversabilityClassMapResult[i], traversabilityClassMap.ptr(x).get());
         }
      }
   }

   @Test
   public void testSpeedToMessage()
   {
      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      TerrainMapData terrainMapData = new TerrainMapData(cellResolution, widthInMeters, 0.0, 0.0);
      int cellsPerAxis = terrainMapData.getCellsPerAxis();

      try (Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(0));
           Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
           Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
           Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
           Mat traversabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));
           Mat traversabilityClassMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5)))
      {
         TerrainMapTools.convertToTerrainMapData(heightMap,
                                                 snapNormalXMap,
                                                 snapNormalYMap,
                                                 snapNormalZMap,
                                                 traversabilityMap,
                                                 traversabilityClassMap,
                                                 new Point3D(0.0, 0.0, 0.0),
                                                 terrainMapData);
      }

      TerrainMapMessage message = new TerrainMapMessage();

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         TerrainMapMessageTools.toMessage(terrainMapData, message);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Terrain Map Data -> Message: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMat = MILLISECOND_TOLERANCE * 5;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
                            "Actual was: " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }

   @Test
   public void testSpeedUnpackMessage()
   {
      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      TerrainMapData terrainMapData = new TerrainMapData(cellResolution, widthInMeters, 0.0, 0.0);
      int cellsPerAxis = terrainMapData.getCellsPerAxis();

      try (Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(0));
           Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
           Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
           Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
           Mat traversabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(4));
           Mat traversabilityClassMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5)))
      {
         TerrainMapTools.convertToTerrainMapData(heightMap,
                                                 snapNormalXMap,
                                                 snapNormalYMap,
                                                 snapNormalZMap,
                                                 traversabilityMap,
                                                 traversabilityClassMap,
                                                 new Point3D(0.0, 0.0, 0.0),
                                                 terrainMapData);
      }

      TerrainMapMessage message = new TerrainMapMessage();
      TerrainMapMessageTools.toMessage(terrainMapData, message);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         TerrainMapMessageTools.unpackMessage(message);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Message -> Terrain Map Data: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMat = MILLISECOND_TOLERANCE * 5;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
                            "Actual was: " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }
}
