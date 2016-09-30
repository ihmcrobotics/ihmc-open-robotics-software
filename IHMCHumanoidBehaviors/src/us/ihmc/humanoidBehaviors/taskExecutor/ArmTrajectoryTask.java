package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ArmTrajectoryTask extends BehaviorTask
{
   private final ArmTrajectoryMessage armTrajectoryMessage;
   private final ArmTrajectoryBehavior armTrajectoryBehavior;



   public ArmTrajectoryTask(ArmTrajectoryMessage armTrajectoryMessage, ArmTrajectoryBehavior armTrajectoryBehavior, DoubleYoVariable yoTime)
   {
      super(armTrajectoryBehavior, yoTime);
      this.armTrajectoryBehavior = armTrajectoryBehavior;
      this.armTrajectoryMessage = armTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      armTrajectoryBehavior.setInput(armTrajectoryMessage);
   }
}
