package us.ihmc.quadrupedRobotics.controller;

public enum QuadrupedControllerEnum
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   FREEZE,
   STEPPING,
   FALL;

   public static QuadrupedControllerEnum[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedControllerEnum fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}