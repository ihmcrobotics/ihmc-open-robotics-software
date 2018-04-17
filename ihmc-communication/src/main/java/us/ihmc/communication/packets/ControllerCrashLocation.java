package us.ihmc.communication.packets;

public enum ControllerCrashLocation
{
   CONTROLLER_READ,
   CONTROLLER_WRITE,
   CONTROLLER_RUN, 
   ESTIMATOR_READ,
   ESTIMATOR_WRITE,
   ESTIMATOR_RUN;

   public static final ControllerCrashLocation[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ControllerCrashLocation fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}