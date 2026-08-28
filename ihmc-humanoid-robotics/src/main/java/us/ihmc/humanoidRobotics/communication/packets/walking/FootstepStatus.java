package us.ihmc.humanoidRobotics.communication.packets.walking;

public enum FootstepStatus
{
   STARTED,
   COMPLETED;

   public static final FootstepStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
