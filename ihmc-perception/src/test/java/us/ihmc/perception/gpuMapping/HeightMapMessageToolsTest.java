package us.ihmc.perception.gpuMapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import perception_msgs.HeightMapMessage;

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapMessageToolsTest
{
   private final int iterations = 1000;
   private final static float MICROSECOND_TOLERANCE = 500.0f;

   /**
    * These values are set to reflect the size of the {@link HeightMapMessage#heights_}.
    * We only allocate so much data in the message so we don't want to store more than that in the {@link HeightMapData#getHeights()}.
    */
   private static final float WIDTH_IN_METERS = 10.0f;
   private static final float CELL_RESOLUTION = 0.2f;

   @Test
   public void testHeightMapMessaging()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(WIDTH_IN_METERS, CELL_RESOLUTION);
      int cellsPerAxis = 2 * centerIndex + 1;
      HeightMapData heightMapData = new HeightMapData(CELL_RESOLUTION, WIDTH_IN_METERS, 0.0, 0.0);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         heightMapData.setHeight(i, 100);
      }

      HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);

      HeightMapData heightMapDataResult = HeightMapMessageTools.unpackMessageToHeightMapData(heightMapMessage);

      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         assertEquals(heightMapData.getHeight(i), heightMapDataResult.getHeight(i));
      }
   }

   @Test
   public void testSpeedUnpackingHeightMapMessage()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(WIDTH_IN_METERS, CELL_RESOLUTION);
      int cellsPerAxis = 2 * centerIndex + 1;
      HeightMapData heightMapData = new HeightMapData(CELL_RESOLUTION, WIDTH_IN_METERS, 0.0, 0.0);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         heightMapData.setHeight(i, 100);
      }

      HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapMessageTools.unpackMessageToHeightMapData(heightMapMessage);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double totalTimeMicroSeconds = (endTime - startTime) / 1_000.0;
      double averageTimeMilliSeconds = totalTimeMillis / iterations;
      double averageTimeMicroSecondsPerIteration = totalTimeMicroSeconds / iterations;

      System.out.printf("Average time per pack of Message -> Height Map Data: %.3f us (%.9f ms)%n",
                        averageTimeMicroSecondsPerIteration,
                        averageTimeMilliSeconds);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedMicrosToPackMessage = MICROSECOND_TOLERANCE;
      Assertions.assertTrue(averageTimeMicroSecondsPerIteration < expectedMicrosToPackMessage,
                            "Actual was: " + averageTimeMicroSecondsPerIteration + ", but the Expected was: " + expectedMicrosToPackMessage);
   }

   @Test
   public void testSpeedOfPackingHeightMapMessage()
   {
      HeightMapData heightMapData = new HeightMapData(CELL_RESOLUTION, WIDTH_IN_METERS, 0.0, 0.0);
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(WIDTH_IN_METERS, CELL_RESOLUTION);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      for (int i = 0; i < totalCells; i++)
      {
         heightMapData.setHeight(i, 1.0f);
      }

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double totalTimeMicroSeconds = (endTime - startTime) / 1_000.0;
      double averageTimeMilliSeconds = totalTimeMillis / iterations;
      double averageTimeMicroSecondsPerIteration = totalTimeMicroSeconds / iterations;

      System.out.printf("Average time per pack of Height Map Data -> Message: %.3f us (%.9f ms)%n",
                        averageTimeMicroSecondsPerIteration,
                        averageTimeMilliSeconds);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedMicrosToPackMessage = MICROSECOND_TOLERANCE;
      Assertions.assertTrue(averageTimeMicroSecondsPerIteration < expectedMicrosToPackMessage,
                            "Actual was: " + averageTimeMicroSecondsPerIteration + ", but the Expected was: " + expectedMicrosToPackMessage);
   }
}