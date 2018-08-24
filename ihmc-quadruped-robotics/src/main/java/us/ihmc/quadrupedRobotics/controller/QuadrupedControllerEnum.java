package us.ihmc.quadrupedRobotics.controller;

public enum QuadrupedControllerEnum
{
   DO_NOTHING_BEHAVIOR,
   STAND_PREP_STATE,
   STAND_READY,
   FREEZE_STATE,
   STAND_TRANSITION_STATE,
   WALKING,
   EXIT_WALKING,
   DIAGNOSTICS,
   CALIBRATION;

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