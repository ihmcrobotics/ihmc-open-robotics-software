package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ObjectWeightTask extends BehaviorTask
{
   private final ObjectWeightPacket objectWeightPacket;
   private final ObjectWeightBehavior objectWeightBehavior;
   
   public ObjectWeightTask(RobotSide robotSide, double weight, ObjectWeightBehavior objectWeightBehavior, DoubleYoVariable yoTime)
   {
      super(objectWeightBehavior, yoTime);
      objectWeightPacket = new ObjectWeightPacket(robotSide, weight);
      this.objectWeightBehavior = objectWeightBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      objectWeightBehavior.setInput(objectWeightPacket);
   }

}
