package us.ihmc.humanoidRobotics.communication.packets.atlas;

public enum AtlasLowLevelControlMode
{
   STAND_PREP, FREEZE;

   public static final AtlasLowLevelControlMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static AtlasLowLevelControlMode fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}