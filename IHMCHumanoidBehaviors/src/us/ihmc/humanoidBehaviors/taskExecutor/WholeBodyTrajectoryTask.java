package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class WholeBodyTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   private final WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;
   

   public WholeBodyTrajectoryTask(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage, WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior)
   {
      this(null, wholebodyTrajectoryMessage, wholebodyTrajectoryBehavior);
   }
   
   public WholeBodyTrajectoryTask(E stateEnum,WholeBodyTrajectoryMessage wholebodyTrajectoryMessage, WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior)
   {
      super(stateEnum,wholebodyTrajectoryBehavior);
      this.wholebodyTrajectoryBehavior = wholebodyTrajectoryBehavior;
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }
   
   @Override
   protected void setBehaviorInput()
   {
      wholebodyTrajectoryBehavior.setInput(wholebodyTrajectoryMessage);
   }
}
