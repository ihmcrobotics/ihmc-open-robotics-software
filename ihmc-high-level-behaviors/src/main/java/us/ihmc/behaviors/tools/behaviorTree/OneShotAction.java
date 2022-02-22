package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Run action only first tick, return RUNNING.
 * No feedback. Task always considered successful.
 * Resettable to run the task again.
 */
public class OneShotAction extends BehaviorTreeAction
{
   private final Runnable action;
   private boolean hasRunOnce = false;

   public OneShotAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      if (!hasRunOnce)
      {
         action.run();
         hasRunOnce = true;
         return RUNNING; // need to allow BT to start from root again
      }
      else
      {
         return SUCCESS;
      }
   }

   public void reset()
   {
      hasRunOnce = false;
   }
}
