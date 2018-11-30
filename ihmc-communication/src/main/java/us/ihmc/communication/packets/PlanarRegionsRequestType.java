package us.ihmc.communication.packets;

public enum PlanarRegionsRequestType
{
   SINGLE_UPDATE, CONTINUOUS_UPDATE, STOP_UPDATE, CLEAR;

   public static final PlanarRegionsRequestType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PlanarRegionsRequestType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}