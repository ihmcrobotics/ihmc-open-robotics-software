package us.ihmc.perception.steppableRegions;

public enum SnapResult
{
   SNAP_FAILED, CLIFF_TOP, CLIFF_BOTTOM, NOT_ENOUGH_AREA, VALID;

   private static final SnapResult[] values = values();

   public static SnapResult fromByte(int i)
   {
      return values[i];
   }
}
