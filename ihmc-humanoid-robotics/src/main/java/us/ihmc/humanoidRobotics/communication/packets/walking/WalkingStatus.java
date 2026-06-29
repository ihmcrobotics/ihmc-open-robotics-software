package us.ihmc.humanoidRobotics.communication.packets.walking;

import controller_msgs.WalkingStatusMessage;

public enum WalkingStatus
{
   STARTED,
   COMPLETED,
   ABORT_REQUESTED,
   PAUSED,
   RESUMED;

   public static final WalkingStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static WalkingStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
