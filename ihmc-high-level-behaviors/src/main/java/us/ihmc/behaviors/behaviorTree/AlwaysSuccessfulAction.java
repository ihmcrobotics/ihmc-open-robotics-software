package us.ihmc.behaviors.behaviorTree;

/**
 * Experimental action. Not sure about this one.
 */
public class AlwaysSuccessfulAction extends BehaviorTreeNodeState
{
   private final Runnable action;

   public AlwaysSuccessfulAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      action.run();
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return null; // FIXME
   }
}
