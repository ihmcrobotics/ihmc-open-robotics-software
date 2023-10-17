package us.ihmc.behaviors.behaviorTree;

/**
 * Core building block of behavior trees. The status that gets passed up the tree.
 */
public enum BehaviorTreeNodeStatus
{
   RUNNING,
   FAILURE,
   SUCCESS;

   public static final BehaviorTreeNodeStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static BehaviorTreeNodeStatus fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
