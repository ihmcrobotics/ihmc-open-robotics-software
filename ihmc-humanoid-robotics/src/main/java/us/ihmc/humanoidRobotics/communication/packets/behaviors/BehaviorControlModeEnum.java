package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum BehaviorControlModeEnum
{
   STOP, PAUSE, RESUME;

   public static final BehaviorControlModeEnum[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static BehaviorControlModeEnum fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}