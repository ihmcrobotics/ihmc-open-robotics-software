package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

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
