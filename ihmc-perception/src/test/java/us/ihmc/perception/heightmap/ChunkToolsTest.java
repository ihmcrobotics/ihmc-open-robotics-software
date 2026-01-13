package us.ihmc.perception.heightmap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.worldModel.ChunkTools;

import static org.junit.jupiter.api.Assertions.*;

public class ChunkToolsTest
{
   private final int iterations = 1000;
   private final static float MILLISECOND_TOLERANCE = 1.0f;

   @RepeatedTest(10)
   public void testRoundTripConvertMatToHeightMapDataAndBack()
   {
      double cellSize = 0.04;
      double terrainWidth = 1.0;
      double centerX = 0.0;
      double centerY = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;
      Point3D centerLocation = new Point3D(centerX, centerY, 0.0);

      Mat originalMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(1000));
      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis, (float) (cellSize * cellsPerAxis));
      ChunkTools.convertToChunk(originalMat, chunkData, centerLocation, (float) terrainWidth, (float) cellSize);

      Mat newData = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      ChunkTools.convertToMat(newData, chunkData);

      for (int x = 0; x < cellsPerAxis; x++)
      {
         for (int y = 0; y < cellsPerAxis; y++)
         {
            assertEquals(originalMat.ptr(x, y).getFloat(), newData.ptr(x, y).getFloat());
         }
      }
   }

   @Test
   public void testSpeedConvertChunkToMat()
   {
      double cellSize = 0.01;
      double terrainWidth = 5.0;
      double centerX = 0.0;
      double centerY = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis, (float) (cellSize * cellsPerAxis));
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            chunkData.setHeight(i, 1.0f);
         }
      }

      Mat chunkMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         ChunkTools.convertToMat(chunkMap, chunkData);
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

      double cellSize = 0.01;
      double terrainWidth = 5.0;
      double centerX = 0.0;
      double centerY = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat chunkMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            chunkMap.ptr(i, j).putFloat(1.0f);
         }
      }

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis, (float) (cellSize * cellsPerAxis));

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         ChunkTools.convertToChunk(chunkMap, chunkData, new Point3D(0.0, 0.0, 0.0), (float) terrainWidth, (float) cellSize);
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
}
