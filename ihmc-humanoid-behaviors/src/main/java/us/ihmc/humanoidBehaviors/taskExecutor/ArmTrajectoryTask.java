package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class ArmTrajectoryTask extends BehaviorAction
{
   private final ArmTrajectoryMessage armTrajectoryMessage;
   private final ArmTrajectoryBehavior armTrajectoryBehavior;

   public ArmTrajectoryTask(ArmTrajectoryMessage armTrajectoryMessage, ArmTrajectoryBehavior armTrajectoryBehavior)
   {
      super(armTrajectoryBehavior);
      this.armTrajectoryBehavior = armTrajectoryBehavior;
      this.armTrajectoryMessage = armTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      armTrajectoryBehavior.setInput(armTrajectoryMessage);
   }
}
