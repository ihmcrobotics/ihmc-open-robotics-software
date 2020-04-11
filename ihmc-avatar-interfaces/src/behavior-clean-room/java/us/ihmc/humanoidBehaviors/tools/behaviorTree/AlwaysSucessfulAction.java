package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

/**
 * Experimental action. Not sure about this one.
 */
public class AlwaysSucessfulAction implements BehaviorTreeAction
{
   private final Runnable action;

   public AlwaysSucessfulAction(Runnable action)
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
