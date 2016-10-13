package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;

public class ArmTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final ArmTrajectoryMessage armTrajectoryMessage;
   private final ArmTrajectoryBehavior armTrajectoryBehavior;

   public ArmTrajectoryTask(ArmTrajectoryMessage armTrajectoryMessage, ArmTrajectoryBehavior armTrajectoryBehavior)
   {
      this(null,armTrajectoryMessage,armTrajectoryBehavior);
   }
   
   public ArmTrajectoryTask(E stateEnum,ArmTrajectoryMessage armTrajectoryMessage, ArmTrajectoryBehavior armTrajectoryBehavior)
   {
      super(stateEnum,armTrajectoryBehavior);
      this.armTrajectoryBehavior = armTrajectoryBehavior;
      this.armTrajectoryMessage = armTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      armTrajectoryBehavior.setInput(armTrajectoryMessage);
   }
}
