package us.ihmc.behaviors.behaviorTree.trashCan;

public enum InteractionStance
{
   FRONT,
   LEFT,
   RIGHT;

   public static final InteractionStance[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static InteractionStance fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
