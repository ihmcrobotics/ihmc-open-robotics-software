package us.ihmc.perception.heightmap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
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
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMap.ptr(i, j).putFloat((i + j));
         }
      }

      HeightMapMessageTools.toMessage(heightMap, heightMapMessage, new Point3D(0.0, 0.0, 0.0), widthInMeters, cellResolution, 3.2768, 10000, cellsPerAxis);

      // Inside the try-with-resource to avoid memory leak
      try (Mat heightMapResult = HeightMapMessageTools.unpackMessageToMat(heightMapMessage))
      {
         for (int i = 0; i < cellsPerAxis; i++)
         {
            for (int j = 0; j < cellsPerAxis; j++)
            {
               assertEquals(heightMap.ptr(i, j).getFloat(), heightMapResult.ptr(i, j).getFloat());
            }
         }
         // Do nothing
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
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMap.ptr(i, j).putInt(1);
         }
      }

      HeightMapMessageTools.toMessage(heightMap, heightMapMessage, new Point3D(0.0, 0.0, 0.0), widthInMeters, cellResolution, 3.2768, 10000.0, cellsPerAxis);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         // Inside the try-with-resource to avoid memory leak
         try (Mat ignored = HeightMapMessageTools.unpackMessageToMat(heightMapMessage))
         {
            // Do nothing
         }
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIterationInMillis = totalTimeMillis / iterations;

      System.out.printf("Average time per unpack of Message -> Mat: %.3f ms%n", averageTimePerIterationInMillis);

      // This will be machine-dependent, the benchmark for this value came from the cpu on the CI machine.
      float expectedTimeTakenToPackHeightMapMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIterationInMillis < expectedTimeTakenToPackHeightMapMessageFromAMatInMillis,
                            "Actual was : " + averageTimePerIterationInMillis + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMatInMillis);
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
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMap.ptr(i, j).putFloat(1.0f);
         }
      }
      Point3D heightMapCenter = new Point3D(0.0, 0.0, 0.0);

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         HeightMapMessageTools.toMessage(heightMap, heightMapMessage, heightMapCenter, widthInMeters, cellResolution, 3.2768, 10000, cellsPerAxis);
      }

      long endTime = System.nanoTime();
      double totalTimeMillis = (endTime - startTime) / 1_000_000.0;
      double averageTimePerIterationInMillis = totalTimeMillis / iterations;

      System.out.printf("Average time per pack Message -> Mat: %.3f ms%n", averageTimePerIterationInMillis);

      // This will be machine-dependent, the benchmark for this value came from a laptop with a AMD Ryzen 7 5800H cpu.
      float expectedTimeTakenToPackHeightMapMessageFromAMatInMillis = MILLISECOND_TOLERANCE;
      Assertions.assertTrue(averageTimePerIterationInMillis < expectedTimeTakenToPackHeightMapMessageFromAMatInMillis,
                            "Actual was: " + averageTimePerIterationInMillis + ", but the Expected was: " + expectedTimeTakenToPackHeightMapMessageFromAMatInMillis);
   }

   /**
    * The reason this test is deprecated is that the method {@link HeightMapMessageTools#toMessage(HeightMapData)} should not be used
    * This test shows how slow it takes to convert to a message with that method. We want this to happen as fast as possible
    */
   @Deprecated
   @Test
   public void testSpeedOfPackingHeightMapMessageWithHeightMapData()
   {
      float widthInMeters = 10.0f;
      float cellResolution = 0.02f;

      HeightMapData heightMapData = new HeightMapData(cellResolution, widthInMeters, 0.0, 0.0);

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
         HeightMapMessageTools.toMessage(heightMapData);
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