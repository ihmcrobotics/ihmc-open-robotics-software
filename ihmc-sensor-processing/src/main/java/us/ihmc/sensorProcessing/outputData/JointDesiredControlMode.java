package us.ihmc.sensorProcessing.outputData;

public enum JointDesiredControlMode
{
   POSITION,
   VELOCITY,
   EFFORT,
   DISABLED;

   public static final JointDesiredControlMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static JointDesiredControlMode fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}