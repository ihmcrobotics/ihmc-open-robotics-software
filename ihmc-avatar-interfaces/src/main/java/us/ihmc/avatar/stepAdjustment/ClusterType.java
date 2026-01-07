package us.ihmc.avatar.stepAdjustment;

public enum ClusterType
{
   /**
    * MULTI_LINE means open at the end, not closed. This type is often used when projecting a PlanarRegion onto another parallel PlanarRegion.
    */
   MULTI_LINE,
   /**
    * POLYGON is a closed, perhaps concave, polygon.
    */
   POLYGON;

   public static ClusterType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ClusterType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
