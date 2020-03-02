package us.ihmc.humanoidBehaviors.tools.behaviorTree;

public interface BehaviorTreeNode
{
   BehaviorTreeNodeStatus tick();

   public static void checkStatusInNotNull(BehaviorTreeNodeStatus status)
   {
      if (status == null)
      {
         throw new RuntimeException("Behavior tree node status must not be null.");
      }
   }
}
