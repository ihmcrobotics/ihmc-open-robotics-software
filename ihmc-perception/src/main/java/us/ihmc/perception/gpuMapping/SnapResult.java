package us.ihmc.perception.gpuMapping;

public enum SnapResult
{
   VALID, SNAP_FAILED, NOT_ENOUGH_AREA, SQUARED_ERROR, TOO_STEEP, CLIFF_TOP;

   public static final SnapResult[] values = values();

   public static SnapResult fromByte(int i)
   {
      return values[i];
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }
}
