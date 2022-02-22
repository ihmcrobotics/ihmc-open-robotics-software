package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum CurrentBehaviorStatus
{
   NO_BEHAVIOR_RUNNING, BEHAVIOR_RUNNING, BEHAVIOR_PAUSED,BEHAVIOR_FINISHED_FAILED,BEHAVIOR_FINISHED_SUCCESS;
   

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