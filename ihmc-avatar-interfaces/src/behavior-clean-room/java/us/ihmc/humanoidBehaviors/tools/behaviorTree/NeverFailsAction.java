package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class NeverFailsAction implements BehaviorTreeAction
{
   private final Runnable action;
   private BehaviorTreeNodeStatus status = RUNNING;

   public NeverFailsAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (status == RUNNING)
      {
         action.run();
         status = SUCCESS;
      }

      return status;
   }

   @Override
   public void reset()
   {
      status = RUNNING;
   }
}
