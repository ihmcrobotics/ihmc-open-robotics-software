package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerEnum
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   FREEZE,
   STEPPING,
   FALL;

   public static QuadrupedForceControllerEnum[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedForceControllerEnum fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}