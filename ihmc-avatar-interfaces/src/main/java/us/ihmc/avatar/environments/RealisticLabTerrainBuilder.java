package us.ihmc.avatar.environments;

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

   public void addPalletStackReferencedAtNegativeXY(int stackHeight)
   {
      for (int i = 0; i < stackHeight; i++)
      {
         addPalletReferencedAtNegativeXY(0.0, 0.0, i);
      }
   }

   public void addPalletReferencedAtNegativeXY(double x, double y, int stackIndex)
   {
      placeWithOffset(0.0, 0.0, stackIndex * PALLET_HEIGHT, () ->
      {
         addBoxReferencedAtNegativeXYZCorner(new Box3D(PALLET_LENGTH, PALLET_WIDTH, PALLET_HEIGHT));
      });
   }
}
