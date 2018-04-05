package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedSteppingRequestedEvent
{
   REQUEST_STAND,
   REQUEST_STEP,
   REQUEST_SOLE_WAYPOINT;

   public static QuadrupedSteppingRequestedEvent[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedSteppingRequestedEvent fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
