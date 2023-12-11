package us.ihmc.behaviors.behaviorTree;

/**
 * Run action only first tick, return RUNNING.
 * No feedback. Task always considered successful.
 * Resettable to run the task again.
 */
public class OneShotAction extends LocalOnlyBehaviorTreeNodeExecutor
{
   private final Runnable action;
   private boolean hasRunOnce = false;

   public OneShotAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      if (!hasRunOnce)
      {
         action.run();
         hasRunOnce = true;
         return BehaviorTreeNodeStatus.RUNNING; // need to allow BT to start from root again
      }
      else
      {
         return BehaviorTreeNodeStatus.SUCCESS;
      }
   }

   public void reset()
   {
      hasRunOnce = false;
   }
}
