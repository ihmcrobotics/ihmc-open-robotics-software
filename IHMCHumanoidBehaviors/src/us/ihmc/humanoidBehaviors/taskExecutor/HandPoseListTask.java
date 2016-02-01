package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class HandPoseListTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final HandPoseListPacket handPoseListPacket;
   private final HandPoseListBehavior handPoseListBehavior;

   public HandPoseListTask(HandPoseListPacket handPoseListPacket, HandPoseListBehavior handPoseListBehavior, DoubleYoVariable yoTime, double sleepTime)
   {
      super(handPoseListBehavior, yoTime, sleepTime);
      this.handPoseListBehavior = handPoseListBehavior;
      this.handPoseListPacket = handPoseListPacket;
   }

   public HandPoseListTask(HandPoseListPacket handPoseListPacket, HandPoseListBehavior handPoseListBehavior, DoubleYoVariable yoTime)
   {
      super(handPoseListBehavior, yoTime);
      this.handPoseListBehavior = handPoseListBehavior;
      this.handPoseListPacket = handPoseListPacket;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseListBehavior.setInput(handPoseListPacket);
   }
}
