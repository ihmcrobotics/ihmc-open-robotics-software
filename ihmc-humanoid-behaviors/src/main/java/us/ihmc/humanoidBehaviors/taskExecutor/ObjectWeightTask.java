package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.robotics.robotSide.RobotSide;

public class ObjectWeightTask extends BehaviorAction
{
   private final ObjectWeightPacket objectWeightPacket;
   private final ObjectWeightBehavior objectWeightBehavior;

   public ObjectWeightTask(RobotSide robotSide, double weight, ObjectWeightBehavior objectWeightBehavior)
   {
      super(objectWeightBehavior);
      objectWeightPacket = HumanoidMessageTools.createObjectWeightPacket(robotSide, weight);
      this.objectWeightBehavior = objectWeightBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      objectWeightBehavior.setInput(objectWeightPacket);
   }

}
