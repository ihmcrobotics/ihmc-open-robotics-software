package us.ihmc.communication.packets;

public enum ExecutionMode
{
   OVERRIDE,
   QUEUE,
   STREAM;

   public static final ExecutionMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ExecutionMode fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
