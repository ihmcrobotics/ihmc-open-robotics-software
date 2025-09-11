package us.ihmc.footstepPlanning.steppableRegions;

public enum SnapResult
{
   VALID, SNAP_FAILED, NOT_ENOUGH_AREA, SQUARED_ERROR, TOO_STEEP;

   private static final SnapResult[] values = values();

   public static SnapResult fromByte(int i)
   {
      return values[i];
   }
}
