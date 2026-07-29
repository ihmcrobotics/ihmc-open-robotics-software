package us.ihmc.communication.packets;

public enum ExecutionTiming
{
   CONTROL_DURATIONS,
   CONTROL_ABSOLUTE_TIMINGS;

   public static final ExecutionTiming[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ExecutionTiming fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
