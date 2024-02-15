package us.ihmc.avatar.sakeGripper;

public enum SakeHandConfiguration
{
   /** Calibration before usage */
   CALIBRATE,
   /** Resets error message */
   RESET,
   /** Fully opens fingers */
   OPEN,
   /** Moves fingers to closed position. Not for gripping */
   CLOSE,
   /** Removes torque */
   RELEASE,
   /** Go to position with specified torque */
   GOTO_POSITION_WITH_TORQUE,
   /** Close with specified torque */
   GRIP_WITH_TORQUE,
   /** Close with maximum torque */
   GRIP_HARD;

   public final static SakeHandConfiguration[] values = values();

   public static SakeHandConfiguration fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }
}
