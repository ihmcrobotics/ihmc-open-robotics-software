package us.ihmc.perception.gpuMapping;

import org.junit.jupiter.api.Test;
import perception_msgs.HeightMapMessageForController;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapForControllerEncoderTest
{
   private static final double CELL_SIZE = 0.02;
   private static final double MAP_WIDTH = 4.0;
   private static final double CROP_WIDTH = 2.4;

   @Test
   public void testRoundTripThroughCommand()
   {
      Random random = new Random(4552);
      HeightMapData heightMapData = new HeightMapData(CELL_SIZE, MAP_WIDTH, 1.37, -0.52);
      float[] heights = heightMapData.getHeights();
      for (int i = 0; i < heights.length; i++)
      {
         double x = heightMapData.getCellPosition(i).getX();
         // stairs with sensor noise and some unknown cells
         heights[i] = (float) (0.15 * Math.floor(Math.max(0.0, x) / 0.3) + 0.003 * random.nextGaussian() - 1.2);
         if (random.nextDouble() < 0.02)
            heights[i] = Float.NaN;
      }

      HeightMapForControllerEncoder encoder = new HeightMapForControllerEncoder(CROP_WIDTH);
      HeightMapMessageForController message = new HeightMapMessageForController();
      HeightMapCommand command = new HeightMapCommand();

      for (double[] center : new double[][] {{1.4, -0.5}, {2.9, 0.4}, {-5.0, 7.0}})
      {
         encoder.encode(heightMapData, center[0], center[1], -1.2, message);
         command.setFromMessage(message);

         assertEquals(2 * HeightMapTools.computeCenterIndex(CROP_WIDTH, CELL_SIZE) + 1, message.getCellsPerAxis());
         assertTrue(message.calculateSizeBytes(0) < 0.2 * 4 * heights.length);

         double halfCrop = 0.5 * message.getWidthInMeters();
         int checkedCells = 0;
         for (int i = 0; i < heights.length; i++)
         {
            double x = heightMapData.getCellPosition(i).getX();
            double y = heightMapData.getCellPosition(i).getY();
            boolean insideCrop = Math.abs(x - message.getGridCenterX()) < halfCrop + 1e-6 && Math.abs(y - message.getGridCenterY()) < halfCrop + 1e-6;
            double decoded = command.getHeight(x, y);
            if (!insideCrop)
            {
               assertTrue(Double.isNaN(decoded));
               continue;
            }
            checkedCells++;
            if (Float.isNaN(heights[i]))
               assertTrue(Double.isNaN(decoded));
            else
               assertEquals(heights[i], decoded, 0.5 * HeightMapForControllerEncoder.DEFAULT_HEIGHT_RESOLUTION + 1e-5);
         }
         assertEquals(message.getCellsPerAxis() * message.getCellsPerAxis(), checkedCells);
      }
   }
}
