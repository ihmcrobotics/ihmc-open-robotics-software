package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Experimental action. Not sure about this one.
 */
public class AlwaysSuccessfulAction extends BehaviorTreeAction
{
   private final Runnable action;

   public AlwaysSuccessfulAction(Runnable action)
   {
      this.action = action;
      setType(AlwaysSuccessfulAction.class);
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      action.run();
      return SUCCESS;
   }
}
