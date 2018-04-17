package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class PelvisTrajectoryTask extends BehaviorAction
{
   private final PelvisTrajectoryMessage pelvisTrajectoryMessage;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;

   public PelvisTrajectoryTask(PelvisTrajectoryMessage pelvisTrajectoryMessage, PelvisTrajectoryBehavior pelvisTrajectoryBehavior)
   {
      super(pelvisTrajectoryBehavior);
      this.pelvisTrajectoryBehavior = pelvisTrajectoryBehavior;
      this.pelvisTrajectoryMessage = pelvisTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage);
   }
}
