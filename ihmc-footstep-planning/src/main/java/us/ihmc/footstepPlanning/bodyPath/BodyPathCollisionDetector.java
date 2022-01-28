package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

class BodyPathCollisionDetector
{
   private final TIntArrayList squaredUpCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList squaredUpCollisionOffsetsY = new TIntArrayList();

   private final TIntArrayList diagonalCollisionOffsetX = new TIntArrayList();
   private final TIntArrayList diagonalCollisionOffsetY = new TIntArrayList();

   void initialize(double gridResolutionXY, double boxSizeX, double boxSizeY)
   {
      packOffsets(gridResolutionXY, squaredUpCollisionOffsetsX, squaredUpCollisionOffsetsY, boxSizeX, boxSizeY, 0.0);
      packOffsets(gridResolutionXY, diagonalCollisionOffsetX, diagonalCollisionOffsetY, boxSizeX, boxSizeY, Math.toRadians(45.0));
   }

   private static void packOffsets(double gridResolutionXY, TIntArrayList xOffsets, TIntArrayList yOffsets, double boxSizeX, double boxSizeY, double angle)
   {
      xOffsets.clear();
      yOffsets.clear();
      int minMaxOffsetX = (int) (0.5 * boxSizeX / gridResolutionXY);
      int minMaxOffsetY = (int) (0.5 * boxSizeY / gridResolutionXY);

      for (int xi = -minMaxOffsetX; xi <= minMaxOffsetX; xi++)
      {
         for (int yi = -minMaxOffsetY; yi <= minMaxOffsetY; yi++)
         {
            double x = xi * gridResolutionXY;
            double y = yi * gridResolutionXY;

            double xP = Math.cos(angle) * x - Math.sin(angle) * y;
            double yP = Math.sin(angle) * x + Math.cos(angle) * y;
            double eps = 1e-8;

            if (Math.abs(xP) < boxSizeX + eps && Math.abs(yP) < boxSizeY + eps)
            {
               xOffsets.add(xi);
               yOffsets.add(yi);
            }
         }
      }
   }

   boolean collisionDetected(HeightMapData heightMapData, BodyPathLatticePoint latticePoint, int yawIndex, double height, double groundClearance)
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);
      double heightThreshold = height + groundClearance;

      TIntArrayList xOffsets = yawIndex % 2 == 0 ? squaredUpCollisionOffsetsX : diagonalCollisionOffsetX;
      TIntArrayList yOffsets = yawIndex % 2 == 0 ? squaredUpCollisionOffsetsY : diagonalCollisionOffsetY;

      for (int i = 0; i < xOffsets.size(); i++)
      {
         int xQuery = xIndex + computeCollisionOffsetX(i, xOffsets.get(i), yOffsets.get(i));
         int yQuery = yIndex + computeCollisionOffsetY(i, yOffsets.get(i), yOffsets.get(i));
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);
         if (Double.isNaN(heightQuery))
         {
            continue;
         }

         if (heightQuery >= heightThreshold)
         {
            return true;
         }
      }

      return false;
   }

   private static int computeCollisionOffsetX(int yawIndex, int xOffset, int yOffset)
   {
      if (yawIndex == 0 || yawIndex == 1 || yawIndex == 4 || yawIndex == 5)
      {
         return xOffset;
      }
      else
      {
         return -yOffset;
      }
   }

   private static int computeCollisionOffsetY(int yawIndex, int xOffset, int yOffset)
   {
      if (yawIndex == 0 || yawIndex == 1 || yawIndex == 4 || yawIndex == 5)
      {
         return yOffset;
      }
      else
      {
         return xOffset;
      }
   }

   public static void main(String[] args)
   {
      System.out.println(HeightMapTools.coordinateToIndex(0.0499, 0.0, 0.1, HeightMapTools.computeCenterIndex(2.0, 0.1)));
      System.out.println(HeightMapTools.coordinateToIndex(0.0501, 0.0, 0.1, HeightMapTools.computeCenterIndex(2.0, 0.1)));
   }
}
