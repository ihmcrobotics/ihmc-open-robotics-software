package us.ihmc.humanoidRobotics.communication.packets.walking;

public enum FootstepPlanRequestType
{
   START_SEARCH, STOP_SEARCH, UPDATE_START;

   public static final FootstepPlanRequestType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlanRequestType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}