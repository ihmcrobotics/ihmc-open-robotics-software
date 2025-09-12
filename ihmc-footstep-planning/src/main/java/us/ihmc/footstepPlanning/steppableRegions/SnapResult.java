package us.ihmc.footstepPlanning.steppableRegions;

import us.ihmc.euclid.geometry.Plane3D;

public enum SnapResult
{
   VALID, SNAP_FAILED, NOT_ENOUGH_AREA, SQUARED_ERROR, TOO_STEEP, CLIFF_TOP;

   private static final SnapResult[] values = values();

   public static SnapResult fromByte(int i)
   {
      return values[i];
   }
}
