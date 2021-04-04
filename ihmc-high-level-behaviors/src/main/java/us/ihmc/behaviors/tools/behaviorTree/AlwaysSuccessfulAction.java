package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Experimental action. Not sure about this one.
 */
public class AlwaysSuccessfulAction implements BehaviorTreeAction
{
   private final Runnable action;

   public AlwaysSuccessfulAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      action.run();
      return SUCCESS;
   }
}
