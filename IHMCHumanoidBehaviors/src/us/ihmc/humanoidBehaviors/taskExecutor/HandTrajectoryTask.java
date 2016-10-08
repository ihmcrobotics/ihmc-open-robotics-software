package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class HandTrajectoryTask extends BehaviorTask
{
   private final HandTrajectoryMessage handTrajectoryMessage;
   private final HandTrajectoryBehavior handTrajectoryBehavior;

   public HandTrajectoryTask(HandTrajectoryMessage handTrajectoryMessage, HandTrajectoryBehavior handTrajectoryBehavior, DoubleYoVariable yoTime)
   {
      super(handTrajectoryBehavior, yoTime);
      this.handTrajectoryBehavior = handTrajectoryBehavior;
      this.handTrajectoryMessage = handTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      handTrajectoryBehavior.setInput(handTrajectoryMessage);
   }
}
