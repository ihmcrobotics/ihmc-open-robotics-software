package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class WholeBodyTrajectoryTask extends BehaviorAction
{
   private final WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   private final WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   public WholeBodyTrajectoryTask(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage, WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior)
   {
      super(wholebodyTrajectoryBehavior);
      this.wholebodyTrajectoryBehavior = wholebodyTrajectoryBehavior;
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      wholebodyTrajectoryBehavior.setInput(wholebodyTrajectoryMessage);
   }
}
