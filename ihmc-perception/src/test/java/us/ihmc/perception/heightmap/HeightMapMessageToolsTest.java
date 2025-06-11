package us.ihmc.perception.heightmap;

import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class HeightMapMessageToolsTest
{

   @Test
   public void testSpeedOfUnpackMessageToMat()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();
      HeightMapParameters heightMapParameters = new HeightMapParameters();

      int iterations = 1000;
      long startTime = System.nanoTime();

      for (int i = 0; i < 1000; i++)
      {
         Mat heightMap = HeightMapMessageTools.unpackMessageToMat(heightMapMessage, heightMapParameters);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per unpack: %.3f ms%n", averageTimePerIteration);
   }
}
