package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

public enum SakeHandCommandOption
{
   OPEN, CLOSE, GRIP, GOAL_POSITION, SET_GOAL_TORQUE, CALIBRATE;

   public final static SakeHandCommandOption[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public SakeHandCommandOption fromByte(byte commandOptionAsByte)
   {
      return values[commandOptionAsByte];
   }
}
