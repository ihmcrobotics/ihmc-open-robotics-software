package us.ihmc.behaviors.sequence.actions;

public enum KickDoorActionExecutionState
{
   STANDING,
   SWITCHING_TO_KICK_CONTROLLER,
   KICKING,
   SWITCHING_TO_WALKING_CONTROLLER,
   SQUARING_UP;

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
