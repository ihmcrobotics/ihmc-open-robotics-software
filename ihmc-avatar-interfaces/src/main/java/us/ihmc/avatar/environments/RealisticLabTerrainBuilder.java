package us.ihmc.avatar.environments;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.robotics.geometry.PlanarRegionsListBuilder;

/**
 * Aims to build up terrains in terms of the pallets and cinder blocks available in the lab.
 */
public class RealisticLabTerrainBuilder extends PlanarRegionsListBuilder
{
   public static final double PALLET_WIDTH = 1.0;
   public static final double PALLET_LENGTH = 1.0;
   public static final double PALLET_HEIGHT = 0.15;

   public static final double LARGE_CINDER_BLOCK_LENGTH = 0.4;
   public static final double LARGE_CINDER_BLOCK_WIDTH = 0.2;
   public static final double LARGE_CINDER_BLOCK_HEIGHT = 0.22; // TODO: Correct these dimensions

   public static final double MEDIUM_CINDER_BLOCK_LENGTH = 0.4;
   public static final double MEDIUM_CINDER_BLOCK_WIDTH = 0.2;
   public static final double MEDIUM_CINDER_BLOCK_HEIGHT = 0.15;
   public static final double MEDIUM_CINDER_BLOCK_ANGLE = Math.toRadians(-18.0);

   public static final double SMALL_CINDER_BLOCK_LENGTH = 0.4;
   public static final double SMALL_CINDER_BLOCK_WIDTH = 0.2;
   public static final double SMALL_CINDER_BLOCK_HEIGHT = 0.08;
   public static final double SMALL_CINDER_BLOCK_ANGLE = Math.toRadians(-13.0);

   private final boolean removeCracks;

   /**
    * The point based contact algorithm will get it's feet stuck in the cracks,
    * so use removeCracks to prevent that.
    * @param removeCracks
    */
   public RealisticLabTerrainBuilder(boolean removeCracks)
   {
      this.removeCracks = removeCracks;
   }

   public void addGround(double size)
   {
      addXYPlaneSquareReferencedAtCenter(size, size);
   }

   public void addPalletStackReferencedAtNegativeXY(int stackHeight, Runnable placementOntoPallet)
   {
      addPalletStackReferencedAtNegativeXY(stackHeight);

      pushOffset(0.0, 0.0, RealisticLabTerrainBuilder.PALLET_HEIGHT);
      placementOntoPallet.run();
      popOffset();
   }

   public void addPalletStackReferencedAtNegativeXY(int stackHeight)
   {
      if (removeCracks)
      {
         addBoxReferencedAtNegativeXYZCorner(new Box3D(PALLET_LENGTH, PALLET_WIDTH, stackHeight * PALLET_HEIGHT));
      }
      else
      {
         for (int i = 0; i < stackHeight; i++)
         {
            addPalletReferencedAtNegativeXY(i);
         }
      }
   }

   private void addPalletReferencedAtNegativeXY(int stackIndex)
   {
      placeWithOffset(0.0, 0.0, stackIndex * PALLET_HEIGHT, () ->
      {
         addBoxReferencedAtNegativeXYZCorner(new Box3D(PALLET_LENGTH, PALLET_WIDTH, PALLET_HEIGHT));
      });
   }

   public void addLargeCinderBlockGroup(int groupSize)
   {
      if (removeCracks)
      {
         addBoxReferencedAtNegativeXYZCorner(LARGE_CINDER_BLOCK_LENGTH, groupSize * LARGE_CINDER_BLOCK_WIDTH, LARGE_CINDER_BLOCK_HEIGHT);
      }
      else
      {
         for (int i = 0; i < groupSize; i++)
         {
            placeWithOffset(0.0, LARGE_CINDER_BLOCK_WIDTH * i, () ->
            {
               addBoxReferencedAtNegativeXYZCorner(LARGE_CINDER_BLOCK_LENGTH, LARGE_CINDER_BLOCK_WIDTH, LARGE_CINDER_BLOCK_HEIGHT);
            });
         }
      }
   }

   public void addMediumCinderBlockGroup(int groupSize)
   {
      if (removeCracks)
      {
         addBoxReferencedAtNegativeXYZCorner(MEDIUM_CINDER_BLOCK_LENGTH, MEDIUM_CINDER_BLOCK_WIDTH * groupSize, MEDIUM_CINDER_BLOCK_HEIGHT);
      }
      else
      {
         for (int i = 0; i < groupSize; i++)
         {
            placeWithOffset(0.0, MEDIUM_CINDER_BLOCK_WIDTH * i, () ->
            {
               addBoxReferencedAtNegativeXYZCorner(MEDIUM_CINDER_BLOCK_LENGTH, MEDIUM_CINDER_BLOCK_WIDTH, MEDIUM_CINDER_BLOCK_HEIGHT);
            });
         }
      }
   }

   public void addMediumAngledCinderBlockGroup(int groupSize)
   {
      if (removeCracks)
      {
         placeWithOffset(0.0, 0.0, 0.0, Axis3D.Y, MEDIUM_CINDER_BLOCK_ANGLE, () ->
         {
            addBoxReferencedAtNegativeXYZCorner(MEDIUM_CINDER_BLOCK_LENGTH, groupSize * MEDIUM_CINDER_BLOCK_WIDTH, MEDIUM_CINDER_BLOCK_HEIGHT);
         });
      }
      else
      {
         for (int i = 0; i < groupSize; i++)
         {
            placeWithOffset(0.0, MEDIUM_CINDER_BLOCK_WIDTH * i, 0.0, Axis3D.Y, MEDIUM_CINDER_BLOCK_ANGLE, () ->
            {
               addBoxReferencedAtNegativeXYZCorner(MEDIUM_CINDER_BLOCK_LENGTH, MEDIUM_CINDER_BLOCK_WIDTH, MEDIUM_CINDER_BLOCK_HEIGHT);
            });
         }
      }
   }

   public void addMediumAngledCinderBlockGroup(int groupSize, double zRotation)
   {
      placeWithOffset(MEDIUM_CINDER_BLOCK_LENGTH / 2.0, MEDIUM_CINDER_BLOCK_WIDTH / 2.0, MEDIUM_CINDER_BLOCK_HEIGHT / 2.0, () ->
      {
         placeWithOffset(0.0, 0.0, 0.0, Axis3D.Z, zRotation, () ->
         {
            if (removeCracks)
            {
               double centroidZOffset = -(MEDIUM_CINDER_BLOCK_LENGTH / 2.0) * Math.tan(MEDIUM_CINDER_BLOCK_ANGLE);
               placeWithOffset(0.0, 0.0, centroidZOffset, Axis3D.Y, MEDIUM_CINDER_BLOCK_ANGLE, () ->
               {
                  addBoxReferencedAtCenter(MEDIUM_CINDER_BLOCK_LENGTH, groupSize * MEDIUM_CINDER_BLOCK_WIDTH, MEDIUM_CINDER_BLOCK_HEIGHT);
               });
            }
            else
            {
               for (int i = 0; i < groupSize; i++)
               {
                  double centroidZOffset = -(MEDIUM_CINDER_BLOCK_LENGTH / 2.0) * Math.tan(MEDIUM_CINDER_BLOCK_ANGLE);
                  placeWithOffset(0.0, MEDIUM_CINDER_BLOCK_WIDTH * i, centroidZOffset, Axis3D.Y, MEDIUM_CINDER_BLOCK_ANGLE, () ->
                  {
                     addBoxReferencedAtCenter(MEDIUM_CINDER_BLOCK_LENGTH, MEDIUM_CINDER_BLOCK_WIDTH, MEDIUM_CINDER_BLOCK_HEIGHT);
                  });
               }
            }
         });
      });
   }
   public void addSmallAngledCinderBlockGroup(int groupSize, double zRotation)
   {
      placeWithOffset(SMALL_CINDER_BLOCK_LENGTH / 2.0, SMALL_CINDER_BLOCK_WIDTH / 2.0, SMALL_CINDER_BLOCK_HEIGHT / 2.0, () ->
      {
         placeWithOffset(0.0, 0.0, 0.0, Axis3D.Z, zRotation, () ->
         {
            if (removeCracks)
            {
               double centroidZOffset = -(SMALL_CINDER_BLOCK_LENGTH / 2.0) * Math.tan(SMALL_CINDER_BLOCK_ANGLE);
               placeWithOffset(0.0, 0.0, centroidZOffset, Axis3D.Y, SMALL_CINDER_BLOCK_ANGLE, () ->
               {
                  addBoxReferencedAtCenter(SMALL_CINDER_BLOCK_LENGTH, groupSize * SMALL_CINDER_BLOCK_WIDTH, SMALL_CINDER_BLOCK_HEIGHT);
               });
            }
            else
            {
               for (int i = 0; i < groupSize; i++)
               {
                  double centroidZOffset = -(SMALL_CINDER_BLOCK_LENGTH / 2.0) * Math.tan(SMALL_CINDER_BLOCK_ANGLE);
                  placeWithOffset(0.0, SMALL_CINDER_BLOCK_WIDTH * i, centroidZOffset, Axis3D.Y, SMALL_CINDER_BLOCK_ANGLE, () ->
                  {
                     addBoxReferencedAtCenter(SMALL_CINDER_BLOCK_LENGTH, SMALL_CINDER_BLOCK_WIDTH, SMALL_CINDER_BLOCK_HEIGHT);
                  });
               }
            }
         });
      });
   }

   public void addSmallCinderBlockGroup(int groupSize)
   {
      if (removeCracks)
      {
         addBoxReferencedAtNegativeXYZCorner(SMALL_CINDER_BLOCK_LENGTH, SMALL_CINDER_BLOCK_WIDTH * groupSize, SMALL_CINDER_BLOCK_HEIGHT);
      }
      else
      {
         for (int i = 0; i < groupSize; i++)
         {
            placeWithOffset(0.0, SMALL_CINDER_BLOCK_WIDTH * i, () ->
            {
               addBoxReferencedAtNegativeXYZCorner(SMALL_CINDER_BLOCK_LENGTH, SMALL_CINDER_BLOCK_WIDTH, SMALL_CINDER_BLOCK_HEIGHT);
            });
         }
      }
   }

   public void placeSmallCinderBlockGroup(double x, double y, double yaw, int groupSize)
   {
      pushOffset(x, y, 0.0, Axis3D.Z, yaw);
      addSmallCinderBlockGroup(groupSize);
      popOffset();
   }

   public void placeMediumCinderBlockGroup(double x, double y, double yaw, int groupSize)
   {
      pushOffset(x, y, 0.0, Axis3D.Z, yaw);
      addMediumCinderBlockGroup(groupSize);
      popOffset();
   }

   public void placeLargeCinderBlockGroup(double x, double y, double yaw, int groupSize)
   {
      pushOffset(x, y, 0.0, Axis3D.Z, yaw);
      addLargeCinderBlockGroup(groupSize);
      popOffset();
   }

   public void placeMediumAngledCinderBlockGroup(double x, double y, double yaw, int groupSize)
   {
      pushOffset(x, y, 0.0, Axis3D.Z, yaw);
      addMediumAngledCinderBlockGroup(groupSize);
      popOffset();
   }
}
