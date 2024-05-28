package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;

/* package-private */
class BodyPathCollisionDetector
{
   /* Offsets of height map cells inside a box rotated at 0 (index 0), pi/8 (index 1) and pi/4 (index 2) radians */
   final TIntArrayList[] xOffsets = new TIntArrayList[]{new TIntArrayList(), new TIntArrayList(), new TIntArrayList()};
   final TIntArrayList[] yOffsets = new TIntArrayList[]{new TIntArrayList(), new TIntArrayList(), new TIntArrayList()};

   void initialize(double gridResolutionXY, double boxSizeX, double boxSizeY)
   {
      for (int i = 0; i < 3; i++)
      {
         packOffsets(gridResolutionXY, xOffsets[i], yOffsets[i], boxSizeX, boxSizeY, i * Math.PI / 8.0);
      }
   }

   /**
    * Yaw index (0-15) represents the angle of the collision box and corresponds to a yaw rotation of (pi * yawIndex / 8)
    */
   boolean collisionDetected(HeightMapData heightMapData, BodyPathLatticePoint latticePoint, int yawIndex, double height, double groundClearance)
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);
      double heightThreshold = height + groundClearance;

      TIntArrayList xOffsets = getOffsets(yawIndex, this.xOffsets);
      TIntArrayList yOffsets = getOffsets(yawIndex, this.yOffsets);

      for (int i = 0; i < xOffsets.size(); i++)
      {
         int xQuery = xIndex + computeCollisionOffsetX(yawIndex, xOffsets.get(i), yOffsets.get(i));
         int yQuery = yIndex + computeCollisionOffsetY(yawIndex, xOffsets.get(i), yOffsets.get(i));
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

   private TIntArrayList getOffsets(int yawIndex, TIntArrayList[] offsets)
   {
      if (yawIndex % 4 == 0)
      {
         return offsets[0];
      }
      else if (yawIndex % 2 == 0)
      {
         return offsets[2];
      }
      else
      {
         return offsets[1];
      }
   }

   /*
    * Helper methods for computing/accessing cells in a square when searching on an eight-connected grid
    */
   static void packOffsets(double gridResolutionXY, TIntArrayList xOffsets, TIntArrayList yOffsets, double boxSizeX, double boxSizeY, double angle)
   {
      xOffsets.clear();
      yOffsets.clear();
      int minMaxOffset = (int) Math.ceil(0.5 * EuclidCoreTools.norm(boxSizeX, boxSizeY) / gridResolutionXY);

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
      /* Box is symmetric so treat rotation by pi the same */
      int yawIndexMod = yawIndex % 8;

      if (yawIndexMod == 0 || yawIndexMod == 1 || yawIndexMod == 2)
      { // rotate by 0
         return xOffset;
      }
      else if (yawIndexMod == 3)
      { // reflect across x = y
         return yOffset;
      }
      else if (yawIndexMod == 4 || yawIndexMod == 5 || yawIndexMod == 6)
      { // rotate by 90
         return -yOffset;
      }
      else if (yawIndexMod == 7)
      { // reflect across x axis
         return xOffset;
      }

      throw new RuntimeException("Invalid yaw index " + yawIndex);
   }

   static int computeCollisionOffsetY(int yawIndex, int xOffset, int yOffset)
   {
      /* Box is symmetric so treat rotation by pi the same */
      int yawIndexMod = yawIndex % 8;

      if (yawIndexMod == 0 || yawIndexMod == 1 || yawIndexMod == 2)
      { // rotate by 0
         return yOffset;
      }
      else if (yawIndexMod == 3)
      { // reflect across x = y
         return xOffset;
      }
      else if (yawIndexMod == 4 || yawIndexMod == 5 || yawIndexMod == 6)
      { // rotate by 90
         return xOffset;
      }
      else if (yawIndexMod == 7)
      { // reflect across x axis
         return -yOffset;
      }

      throw new RuntimeException("Invalid yaw index " + yawIndex);
   }
}
