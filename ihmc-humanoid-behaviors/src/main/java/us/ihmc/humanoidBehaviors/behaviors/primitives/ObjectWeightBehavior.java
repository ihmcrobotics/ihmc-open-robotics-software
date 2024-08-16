package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.ObjectWeightPacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ObjectWeightBehavior extends AbstractBehavior
{
   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet" + behaviorName, registry);
   private final YoBoolean packetAvailable = new YoBoolean("packetAvailable" + behaviorName, registry);
   private ObjectWeightPacket objectWeightPacket;
   private ROS2PublisherBasics<ObjectWeightPacket> publisher;

   public ObjectWeightBehavior(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ros2Node);

      publisher = createPublisher(ObjectWeightPacket.class, ROS2Tools.IHMC_ROOT);
   }

   @Override
   public void doControl()
   {
      if (isPaused())
      {
         return;
      }

      if (packetAvailable.getBooleanValue())
      {
         publisher.publish(objectWeightPacket);
         packetAvailable.set(false);
      }
   }

   public void setInput(ObjectWeightPacket packet)
   {
      objectWeightPacket = packet;
      packetAvailable.set(true);
      hasInputBeenSet.set(true);
   }

   @Override
   public boolean isDone()
   {
      return hasInputBeenSet() && !packetAvailable.getBooleanValue();
   }

   @Override
   public void onBehaviorExited()
   {
      hasInputBeenSet.set(false);
   }

   @Override
   public void onBehaviorEntered()
   {
      hasInputBeenSet.set(false);
      packetAvailable.set(false);
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}
