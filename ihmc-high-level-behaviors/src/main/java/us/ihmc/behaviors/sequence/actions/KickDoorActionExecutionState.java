package us.ihmc.behaviors.sequence.actions;

public enum KickDoorActionExecutionState
{
   STANDING,
   SWITCHING_HIGH_LEVEL_CONTROLLER_TO_KICK_CONTROLLER,
   EXECUTING_KICKING,
   SWITCHING_HIGH_LEVEL_CONTROLLER_TO_WALKING_CONTROLLER,
   KICK_COMPLETED;

   public static final KickDoorActionExecutionState[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static KickDoorActionExecutionState fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
