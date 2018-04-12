package us.ihmc.quadrupedRobotics.controller;

public enum QuadrupedControllerRequestedEvent
{
   REQUEST_DO_NOTHING,
   REQUEST_STAND_PREP,
   REQUEST_FREEZE,
   REQUEST_STEPPING,
   REQUEST_FALL;

   public static QuadrupedControllerRequestedEvent[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedControllerRequestedEvent fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
