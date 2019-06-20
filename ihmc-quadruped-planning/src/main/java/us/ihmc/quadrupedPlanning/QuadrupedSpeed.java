package us.ihmc.quadrupedPlanning;

public enum QuadrupedSpeed
{
   SLOW, MEDIUM, FAST;

   public final static QuadrupedSpeed[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedSpeed fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
