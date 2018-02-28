package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum CurrentBehaviorStatus
{
   NO_BEHAVIOR_RUNNING, BEHAVIOS_RUNNING, BEHAVIOR_PAUSED;

   public static final CurrentBehaviorStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static CurrentBehaviorStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}