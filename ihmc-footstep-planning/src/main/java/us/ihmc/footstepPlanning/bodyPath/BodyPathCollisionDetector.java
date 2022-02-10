package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.heightMap.HeightMapData;
import us.ihmc.robotics.heightMap.HeightMapTools;

class BodyPathCollisionDetector
{
   final TIntArrayList zeroDegCollisionOffsetsX = new TIntArrayList();
   final TIntArrayList zeroDegCollisionOffsetsY = new TIntArrayList();

   final TIntArrayList fourtyFiveDegCollisionOffsetsX = new TIntArrayList();
   final TIntArrayList fourtyFiveDegCollisionOffsetsY = new TIntArrayList();

   final TIntArrayList twentyTwoDegCollisionOffsetsX = new TIntArrayList();
   final TIntArrayList twentyTwoDegCollisionOffsetsY = new TIntArrayList();

   void initialize(double gridResolutionXY, double boxSizeX, double boxSizeY)
   {
      packOffsets(gridResolutionXY, zeroDegCollisionOffsetsX, zeroDegCollisionOffsetsY, boxSizeX, boxSizeY, 0.0);
      packOffsets(gridResolutionXY, fourtyFiveDegCollisionOffsetsX, fourtyFiveDegCollisionOffsetsY, boxSizeX, boxSizeY, Math.toRadians(45.0));
      packOffsets(gridResolutionXY, twentyTwoDegCollisionOffsetsX, twentyTwoDegCollisionOffsetsY, boxSizeX, boxSizeY, Math.toRadians(22.5));
   }

   boolean collisionDetected(HeightMapData heightMapData, BodyPathLatticePoint latticePoint, int yawIndex, double height, double groundClearance)
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);
      double heightThreshold = height + groundClearance;

      TIntArrayList xOffsets = getXOffsets(yawIndex);
      TIntArrayList yOffsets = getYOffsets(yawIndex);

      for (int i = 0; i < xOffsets.size(); i++)
      {
         int xQuery = xIndex + computeCollisionOffsetX(i, xOffsets.get(i), yOffsets.get(i));
         int yQuery = yIndex + computeCollisionOffsetY(i, xOffsets.get(i), yOffsets.get(i));
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

   private TIntArrayList getXOffsets(int yawIndex)
   {
      switch (yawIndex)
      {
         case 0:
         case 2:
         case 4:
         case 6:
            return zeroDegCollisionOffsetsX;
         case 1:
         case 3:
         case 5:
         case 7:
            return fourtyFiveDegCollisionOffsetsX;
         case 8:
         case 9:
         case 10:
         case 11:
         case 12:
         case 13:
         case 14:
         case 15:
            return twentyTwoDegCollisionOffsetsX;
      }

      throw new RuntimeException("Yaw index out of range: " + yawIndex);
   }

   private TIntArrayList getYOffsets(int yawIndex)
   {
      switch (yawIndex)
      {
         case 0:
         case 2:
         case 4:
         case 6:
            return zeroDegCollisionOffsetsY;
         case 1:
         case 3:
         case 5:
         case 7:
            return fourtyFiveDegCollisionOffsetsY;
         case 8:
         case 9:
         case 10:
         case 11:
         case 12:
         case 13:
         case 14:
         case 15:
            return twentyTwoDegCollisionOffsetsY;
      }

      throw new RuntimeException("Yaw index out of range: " + yawIndex);
   }

   /*
    * Helper methods for computing/accessing cells in a square when searching on an eight-connected grid
    */
   static void packOffsets(double gridResolutionXY, TIntArrayList xOffsets, TIntArrayList yOffsets, double boxSizeX, double boxSizeY, double angle)
   {
      xOffsets.clear();
      yOffsets.clear();
      int minMaxOffset = (int) (0.5 * EuclidCoreTools.norm(boxSizeX, boxSizeY) / gridResolutionXY);

      for (int xi = -minMaxOffset; xi <= minMaxOffset; xi++)
      {
         for (int yi = -minMaxOffset; yi <= minMaxOffset; yi++)
         {
            double x = xi * gridResolutionXY;
            double y = yi * gridResolutionXY;

            double xP = Math.cos(angle) * x + Math.sin(angle) * y;
            double yP = -Math.sin(angle) * x + Math.cos(angle) * y;
            double eps = 1e-8;

            if ((Math.abs(xP) < 0.5 * boxSizeX + eps) && (Math.abs(yP) < 0.5 * boxSizeY + eps))
            {
               xOffsets.add(xi);
               yOffsets.add(yi);
            }
         }
      }
   }

   static int computeCollisionOffsetX(int yawIndex, int xOffset, int yOffset)
   {
      // rotate by 0
      if (yawIndex == 0 || yawIndex == 1 || yawIndex == 4 || yawIndex == 5 || yawIndex == 8 || yawIndex == 12)
      {
         return xOffset;
      }
      // reflect across x = y
      else if (yawIndex == 9 || yawIndex == 13)
      {
         return yOffset;
      }
      // reflect x axis
      else if (yawIndex == 11 || yawIndex == 15)
      {
         return xOffset;
      }
      // rotate by 90
      else
      {
         return -yOffset;
      }
   }

   static int computeCollisionOffsetY(int yawIndex, int xOffset, int yOffset)
   {
      // rotate by 0
      if (yawIndex == 0 || yawIndex == 1 || yawIndex == 4 || yawIndex == 5 || yawIndex == 8 || yawIndex == 12)
      {
         return yOffset;
      }
      // reflect across x = y
      else if (yawIndex == 9 || yawIndex == 13)
      {
         return xOffset;
      }
      // reflect x axis
      else if (yawIndex == 11 || yawIndex == 15)
      {
         return -yOffset;
      }
      // rotate by 90
      else
      {
         return xOffset;
      }
   }
}
