package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class PelvisOrientationTrajectoryTask extends BehaviorTask
{
   private final PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
   private final PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior;

   public PelvisOrientationTrajectoryTask(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage, DoubleYoVariable yoTime,
         PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior)
   {
      
      super(pelvisOrientationTrajectoryBehavior, yoTime);
      this.pelvisOrientationTrajectoryBehavior = pelvisOrientationTrajectoryBehavior;
      this.pelvisOrientationTrajectoryMessage = pelvisOrientationTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
   }
}
