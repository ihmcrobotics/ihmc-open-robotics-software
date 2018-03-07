package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;

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
