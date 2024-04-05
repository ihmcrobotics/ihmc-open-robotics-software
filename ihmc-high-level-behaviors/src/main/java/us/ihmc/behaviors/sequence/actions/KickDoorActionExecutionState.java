package us.ihmc.behaviors.sequence.actions;

public enum KickDoorActionExecutionState
{
   STANDING,
   PREPARING_KICK_FOOT,
   EXECUTING_KICKING,
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
