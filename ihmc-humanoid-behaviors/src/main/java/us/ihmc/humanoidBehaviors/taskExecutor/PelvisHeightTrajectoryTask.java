package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class PelvisHeightTrajectoryTask extends BehaviorAction
{
   private final PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;

   public PelvisHeightTrajectoryTask(double heightInWorld, PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior, double trajectoryTime)
   {
      super(pelvisHeightTrajectoryBehavior);
      this.pelvisHeightTrajectoryBehavior = pelvisHeightTrajectoryBehavior;
      pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime, heightInWorld);
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
   }

}
