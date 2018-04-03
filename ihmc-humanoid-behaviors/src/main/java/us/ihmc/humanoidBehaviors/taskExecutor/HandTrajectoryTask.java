package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class HandTrajectoryTask extends BehaviorAction
{
   private final HandTrajectoryMessage handTrajectoryMessage;
   private final HandTrajectoryBehavior handTrajectoryBehavior;

   public HandTrajectoryTask(HandTrajectoryMessage handTrajectoryMessage, HandTrajectoryBehavior handTrajectoryBehavior)
   {
      super(handTrajectoryBehavior);
      this.handTrajectoryBehavior = handTrajectoryBehavior;
      this.handTrajectoryMessage = handTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      handTrajectoryBehavior.setInput(handTrajectoryMessage);
   }
}
