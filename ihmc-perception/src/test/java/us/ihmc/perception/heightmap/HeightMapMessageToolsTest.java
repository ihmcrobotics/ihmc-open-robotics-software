package us.ihmc.perception.heightmap;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapMessageToolsTest
{
   private final int iterations = 1000;
   private final static float MILLISECOND_TOLERANCE = 1.0f;

   @Test
   public void testHeightMapMessaging()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellResolution);
      int cellsPerAxis = 2 * centerIndex + 1;
      HeightMapData heightMapData = new HeightMapData(cellResolution, widthInMeters, 0.0, 0.0);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         heightMapData.setHeight(i, 10);
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

      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellResolution);
      int cellsPerAxis = 2 * centerIndex + 1;
      HeightMapData heightMapData = new HeightMapData(cellResolution, widthInMeters, 0.0, 0.0);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
      {
         heightMapData.setHeight(i, 10);
      }

      HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapMessageTools.unpackMessageToHeightMapData(heightMapMessage);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIterationInMillis = totalTimeMillis / iterations;

      System.out.printf("Average time per unpack of Message -> Mat: %.3f ms%n", averageTimePerIterationInMillis);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackHeightMapMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIterationInMillis < expectedTimeTakenToPackHeightMapMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIterationInMillis + ", but the Expected was: "
                            + expectedTimeTakenToPackHeightMapMessageFromAMatInMillis);
   }

   @Test
   public void testSpeedOfPackingHeightMapMessage()
   {
      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      HeightMapData heightMapData = new HeightMapData(cellResolution, widthInMeters, 0.0, 0.0);
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellResolution);
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
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack of Message -> Height Map Data: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      //      float expectedTimeTakenToPackHeightMapMessageFromAMat = 5.0f;
      //      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
      //                            "Actual was: " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }
}