package us.ihmc.footstepPlanning.tools;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.perception.tools.PerceptionDataTools;

public class HeightMapTerrainGeneratorTools
{
   public static void fillWithStepUpBlocks(Mat heightMap, float width, float length, float stepHeight, int numberOfBlocks)
   {
      for (int i = 0; i < numberOfBlocks; i++)
      {
         PerceptionDataTools.fillWithStep(heightMap, new Point2D(-0.5 + i * 0.5, 0), new Point2D(width, length), stepHeight * (i + 1), false);
      }
   }

   public static void fillWithStepDownBlocks(Mat heightMap, float width, float length, float stepHeight, int numberOfBlocks)
   {
      for (int i = 0; i < numberOfBlocks; i++)
      {
         PerceptionDataTools.fillWithStep(heightMap, new Point2D(-0.5 + i * 0.5, 0), new Point2D(width, length), stepHeight * (numberOfBlocks - i), false);
      }
   }

   public static void fillWithStepUpAndDownBlocks(Mat heightMap, float width, float length, float stepHeight, int numberOfBlocks)
   {
      for (int i = 0; i < numberOfBlocks; i++)
      {
         PerceptionDataTools.fillWithStep(heightMap, new Point2D(-0.5 + i * 0.5, 0), new Point2D(width, length), stepHeight * (i + 1), false);
      }
      for (int i = 0; i < numberOfBlocks; i++)
      {
         PerceptionDataTools.fillWithStep(heightMap, new Point2D(-0.5 + (i + numberOfBlocks) * 0.5, 0), new Point2D(width, length), stepHeight * (numberOfBlocks - i), false);
      }
   }

   public static void fillWithSteppingStones(Mat heightMap, float width, float length, float spacing, float stepHeight, int numberOfBlocks)
   {
      for (int i = 0; i < numberOfBlocks; i++)
      {
         PerceptionDataTools.fillWithStep(heightMap, new Point2D(-0.5 + i * 0.5, Math.pow(-1, i % 2) * spacing), new Point2D(width, length), stepHeight, false);
      }
   }
}
