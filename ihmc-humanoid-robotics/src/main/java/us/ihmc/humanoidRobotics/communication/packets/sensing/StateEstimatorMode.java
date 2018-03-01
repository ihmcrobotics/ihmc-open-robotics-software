package us.ihmc.humanoidRobotics.communication.packets.sensing;

public enum StateEstimatorMode
{
   NORMAL, FROZEN;
   
   public static StateEstimatorMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static StateEstimatorMode fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}