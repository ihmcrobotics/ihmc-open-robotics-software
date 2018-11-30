package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum WalkToGoalAction
{
   FIND_PATH, EXECUTE, EXECUTE_UNKNOWN, STOP;

   public static final WalkToGoalAction[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static WalkToGoalAction fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}