package us.ihmc.behaviors.tools.behaviorTree;

/**
 * A behavior tree action is a leaf node. Actions are the "end effectors" of behavior trees.
 */
public abstract class BehaviorTreeAction extends BehaviorTreeNode
{
   // TODO: What is common among actions?

   public BehaviorTreeAction()
   {
      setType(BehaviorTreeAction.class);
   }
}
