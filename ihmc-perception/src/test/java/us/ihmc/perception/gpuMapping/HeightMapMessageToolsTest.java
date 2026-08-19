package us.ihmc.perception.gpuMapping;

import static org.junit.jupiter.api.Assertions.*;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.global.opencv_core;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.HeightMapMessage;

import java.util.Random;

public class HeightMapMessageToolsTest
{
   private final int iterations = 1000;
   private final static float MICROSECOND_TOLERANCE = 500.0f;

   /**
    * Packing reaches for a JavaCPP {@code FloatPointer} before it touches any OpenCV class, so
    * nothing here loads the native library on its own and these tests only pass when some other test
    * in the same JVM happened to load it first.
    */
   @BeforeAll
   public static void loadNativeLibrary()
   {
      Loader.load(opencv_core.class);
   }

   /**
    * These values are set to reflect the size of the {@link HeightMapMessage#heights_}.
    * We only allocate so much data in the message so we don't want to store more than that in the {@link HeightMapData#getHeights()}.
    */
   private static final float WIDTH_IN_METERS = 10.0f;
   private static final float CELL_RESOLUTION = 0.2f;

   /**
    * The configured map has to fit in the message on its worst day. {@link HeightMapMessageTools#toMessage}
    * PNGs the heights, but PNG is being handed the bit patterns of floats, which do not compress at
    * all once the terrain is anything but flat — measure with noise and the compressed size lands
    * slightly above the raw four bytes a cell. So the real ceiling is about 63,000 cells whatever the
    * terrain, and widening the map or shrinking its cells past that makes publishing throw, on the
    * robot, only once it sees real ground. Trade width against cell size to stay under this.
    */
   @Test
   public void theConfiguredMapFitsInTheMessageWhenNothingCompresses()
   {
      HeightMapParameters parameters = new HeightMapParameters();
      int cellsPerAxis = 2 * HeightMapTools.computeCenterIndex(parameters.getGlobalWidthInMeters(), parameters.getCellSize()) + 1;
      HeightMapData heightMapData = new HeightMapData((float) parameters.getCellSize(), (float) parameters.getGlobalWidthInMeters(), 0.0, 0.0);

      Random random = new Random(34567L);
      for (int i = 0; i < cellsPerAxis * cellsPerAxis; i++)
         heightMapData.setHeight(i, (float) random.nextGaussian());

      HeightMapMessage message = new HeightMapMessage();
      HeightMapMessageTools.toMessage(heightMapData, message);

      int maxSize = message.getHeights().getMaxSize();
      assertTrue(message.getHeights().size() < maxSize,
                 "A " + parameters.getGlobalWidthInMeters() + "m map of " + parameters.getCellSize() + "m cells is " + cellsPerAxis
                 + " cells per axis, which packs to " + message.getHeights().size() + " bytes of the " + maxSize + " the message allows");
   }

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