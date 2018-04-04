package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerRequestedEvent
{
   REQUEST_DO_NOTHING,
   REQUEST_STAND_PREP,
   REQUEST_FREEZE,
   REQUEST_STEPPING,
   REQUEST_FALL;

   public static QuadrupedForceControllerRequestedEvent[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedForceControllerRequestedEvent fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
