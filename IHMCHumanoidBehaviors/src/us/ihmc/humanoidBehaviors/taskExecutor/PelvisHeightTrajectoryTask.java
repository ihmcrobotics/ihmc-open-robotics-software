package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;

public class PelvisHeightTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;

   public PelvisHeightTrajectoryTask(double heightInWorld, PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior, double trajectoryTime)
   {
      this(null, heightInWorld, pelvisHeightTrajectoryBehavior, trajectoryTime);
   }

   public PelvisHeightTrajectoryTask(E stateEnum, double heightInWorld, PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior, double trajectoryTime)
   {
      super(stateEnum, pelvisHeightTrajectoryBehavior);
      this.pelvisHeightTrajectoryBehavior = pelvisHeightTrajectoryBehavior;
      pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(trajectoryTime, heightInWorld);
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
   }

}
