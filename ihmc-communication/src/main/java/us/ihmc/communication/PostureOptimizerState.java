package us.ihmc.communication;

public enum PostureOptimizerState
{
   /* Low margin, high sensitivity */
   OPTIMIZER,
   /* Medium margin or low margin + low sensitivity */
   FREEZE,
   /* High margin */
   NOMINAL;

   private static final PostureOptimizerState[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PostureOptimizerState fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
