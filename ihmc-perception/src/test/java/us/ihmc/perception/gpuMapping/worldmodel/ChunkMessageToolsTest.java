package us.ihmc.perception.gpuMapping.worldmodel;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.worldModel.ChunkMessageTools;

import static org.junit.jupiter.api.Assertions.*;

public class ChunkMessageToolsTest
{
   private final int iterations = 1000;
   private final static float MILLISECOND_TOLERANCE = 2.0f;

   private static final float WIDTH_IN_METERS = 1.0f;
   private static final float CELL_RESOLUTION = 0.02f;

   /**
    * We are testing that when a message is packed and sent. That when its unpacked, its values are all the same.
    * So we pack a {@link ChunkMessage} and check that when we unpack the {@link ChunkMessage} all the values match
    * the original data.
    */
   @RepeatedTest(10)
   public void testChunkMapMessaging()
   {
      ChunkMessage chunkMessage = new ChunkMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(WIDTH_IN_METERS, CELL_RESOLUTION);
      int cellsPerAxis = 2 * centerIndex;
      Chunk chunk = new Chunk(0.0f, 0.0f, CELL_RESOLUTION, cellsPerAxis);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         chunk.setHeight(i, 100);
      }

      ChunkMessageTools.toMessage(chunk, chunkMessage);

      Chunk chunkResult = new Chunk(chunkMessage.getOriginX(), chunkMessage.getOriginY(), chunkMessage.getCellSizeInMeters(), chunkMessage.getCellsPerAxis());

      ChunkMessageTools.unpackMessageToChunk(chunkMessage, chunkResult);

      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         assertEquals(chunk.getChunkHeights()[i], chunkResult.getChunkHeights()[i]);
      }
   }

   @Test
   public void testSpeedUnpackingChunkMessage()
   {
      ChunkMessage chunkMessage = new ChunkMessage();

      float cellSize = 0.02f;
      float terrainWidth = 1.0f;
      float centerX = 0.0f;
      float centerY = 0.0f;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis);

      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         chunkData.setHeight(i, 100);
      }

      ChunkMessageTools.toMessage(chunkData, chunkMessage);

      long startTime = System.nanoTime();

      Chunk chunkResult = new Chunk(centerX, centerY, cellSize, cellsPerAxis);

      for (int i = 0; i < iterations; i++)
      {
         ChunkMessageTools.unpackMessageToChunk(chunkMessage, chunkResult);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimeMilliSeconds = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Message -> Height Map Data: %.3f us (%.9f ms)%n", averageTimeMilliSeconds, averageTimeMilliSeconds);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedMicrosToPackMessage = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimeMilliSeconds < expectedMicrosToPackMessage,
                            "Actual was: " + averageTimeMilliSeconds + ", but the Expected was: " + expectedMicrosToPackMessage);
   }

   @Test
   public void testSpeedOfPackingChunkMessage()
   {
      float cellSize = 0.02f;
      float terrainWidth = 1.0f;
      float centerX = 0.0f;
      float centerY = 0.0f;

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, cellSize);
      int cellsPerAxis = 2 * centerIndex + 1;

      Chunk chunkData = new Chunk(centerX, centerY, cellSize, cellsPerAxis);

      ChunkMessage chunkMessage = new ChunkMessage();

      int totalCells = cellsPerAxis * cellsPerAxis;

      for (int i = 0; i < totalCells; i++)
      {
         chunkData.setHeight(i, 1.0f);
      }

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         ChunkMessageTools.toMessage(chunkData, chunkMessage);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimeMilliSeconds = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Height Map Data -> Message: %.3f us (%.9f ms)%n", averageTimeMilliSeconds, averageTimeMilliSeconds);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedMicrosToPackMessage = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimeMilliSeconds < expectedMicrosToPackMessage,
                            "Actual was: " + averageTimeMilliSeconds + ", but the Expected was: " + expectedMicrosToPackMessage);
   }
}
