package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class PelvisHeightTrajectoryTask extends BehaviorTask
{
   private final PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;

   public PelvisHeightTrajectoryTask(double heightInWorld, DoubleYoVariable yoTime, PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior, double trajectoryTime)
   {
    
      super(pelvisHeightTrajectoryBehavior, yoTime);
      this.pelvisHeightTrajectoryBehavior = pelvisHeightTrajectoryBehavior;
      pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(trajectoryTime, heightInWorld);
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
   }


}
