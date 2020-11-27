package us.ihmc.humanoidBehaviors.tools.behaviorTree;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public interface BehaviorTreeNode
{
   default double evaluateUtility()
   {
      return 1.0;
   }

   BehaviorTreeNodeStatus tick();

   public static void checkStatusInNotNull(BehaviorTreeNodeStatus status)
   {
      if (status == null)
      {
         throw new RuntimeException("Behavior tree node status must not be null.");
      }
   }
}
