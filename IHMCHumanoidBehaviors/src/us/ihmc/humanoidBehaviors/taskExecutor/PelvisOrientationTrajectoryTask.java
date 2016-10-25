package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class PelvisOrientationTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
   private final PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior;

   public PelvisOrientationTrajectoryTask(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage,
         PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior)
   {
      this(null, pelvisOrientationTrajectoryMessage, pelvisOrientationTrajectoryBehavior);
   }

   public PelvisOrientationTrajectoryTask(E stateEnum, PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage,
         PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior)
   {
      super(stateEnum, pelvisOrientationTrajectoryBehavior);
      this.pelvisOrientationTrajectoryBehavior = pelvisOrientationTrajectoryBehavior;
      this.pelvisOrientationTrajectoryMessage = pelvisOrientationTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
   }
}
