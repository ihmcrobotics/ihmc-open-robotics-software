package us.ihmc.humanoidRobotics.communication.packets.walking;

public enum LoadBearingRequest
{
   LOAD,
   UNLOAD;

   public static final LoadBearingRequest[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static LoadBearingRequest fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
