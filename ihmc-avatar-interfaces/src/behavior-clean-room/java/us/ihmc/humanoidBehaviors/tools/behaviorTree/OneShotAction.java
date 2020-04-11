package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Run action only first tick, return RUNNING.
 * No feedback. Task always considered successful.
 * Resettable to run the task again.
 */
public class OneShotAction implements BehaviorTreeAction
{
   private final Runnable action;
   private boolean hasRunOnce = false;

   public OneShotAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
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
