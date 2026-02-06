package us.ihmc.perception.gpuMapping.worldmodel;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.worldModel.ChunkTools;

import static org.junit.jupiter.api.Assertions.*;

public class ChunkToolsTest
{
   private final static float MILLISECOND_TOLERANCE = 1.0f;
   private final int iterations = 1000;

   @RepeatedTest(10)
   public void testRoundTripConvertMatToChunkAndBack()
   {
      float cellSize = 0.04f;
      int terrainWidth = 1;
      float centerX = 0.0f;
      float centerY = 0.0f;

      int cellsPerAxis = (int) (terrainWidth / cellSize);
      Point3D centerLocation = new Point3D(centerX, centerY, 0.0);

      Mat originalMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(1000));
      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis);
      ChunkTools.convertToChunk(originalMat, chunkData, centerLocation, terrainWidth, cellSize);

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
      float cellSize = 0.01f;
      float terrainWidth = 5.0f;
      float centerX = 0.0f;
      float centerY = 0.0f;

      int cellsPerAxis = (int) (terrainWidth / cellSize);

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis);
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

      System.out.printf("Average time per pack [ Chunk -> Mat ]: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackChunkMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackChunkMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackChunkMessageFromAMatInMillis);
   }

   @Test
   public void testSpeedConvertMatToChunkData()
   {

      float cellSize = 0.1f;
      float terrainWidth = 5.0f;
      float centerX = 0.0f;
      float centerY = 0.0f;

      int cellsPerAxis = (int) (terrainWidth / cellSize);

      Mat chunkMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(1));

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         ChunkTools.convertToChunk(chunkMap, chunkData, new Point3D(0.0, 0.0, 0.0), terrainWidth, cellSize);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack [ Mat -> ChunkData ]: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackChunkMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackChunkMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackChunkMessageFromAMatInMillis);
   }
}
