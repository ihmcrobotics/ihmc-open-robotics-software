package us.ihmc.perception.heightmap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;

public class HeightMapMessageToolsTest
{
   @Test
   public void testSpeedUnpackingHeightMapMessage()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();
      HeightMapParameters heightMapParameters = new HeightMapParameters();

      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellResolution);
      int cellsPerAxis = 2 * centerIndex + 1;
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMap.ptr(i, j).putFloat(1.0f);
         }
      }

      HeightMapMessageTools.toMessage(heightMap, heightMapMessage, new Point3D(0.0, 0.0, 0.0), widthInMeters, cellResolution);

      int iterations = 1000;
      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         try (Mat heightMapResult = HeightMapMessageTools.unpackMessageToMat(heightMapMessage, heightMapParameters))
         {
            // Do nothing
         }
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMat = 2.0f;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }

   /**
    * This test helps ensure that the speed of these messages isn't causing problems in the height map pipeline.
    */
   @Test
   public void testSpeedOfPackingHeightMapMessage()
   {
      HeightMapMessage heightMapMessage = new HeightMapMessage();

      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellResolution);
      int cellsPerAxis = 2 * centerIndex + 1;
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMap.ptr(i, j).putFloat(1.0f);
         }
      }

      int iterations = 500;
      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapMessageTools.toMessage(heightMap, heightMapMessage, new Point3D(0.0, 0.0, 0.0), widthInMeters, cellResolution);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIteration = totalTimeMillis / iterations;

      System.out.printf("Average time per pack: %.3f ms%n", averageTimePerIteration);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMat = 3.0f;
      Assertions.assertTrue(averageTimePerIteration < expectedTimeTakenToPackHeightMapMessageFromAMat,
                            "Actual was : " + averageTimePerIteration + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMat);
   }
}
