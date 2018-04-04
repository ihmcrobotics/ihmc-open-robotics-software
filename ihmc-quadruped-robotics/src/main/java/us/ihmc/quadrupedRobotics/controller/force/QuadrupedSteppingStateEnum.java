package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedSteppingStateEnum
{
   STAND,
   STEP,
   SOLE_WAYPOINT;

   public static QuadrupedSteppingStateEnum[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedSteppingStateEnum fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
