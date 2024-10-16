package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

public class FootLoadBearingBehaviorTest
{
   @Test
   public void testSetInput()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "test_set_input");
      FootLoadBearingBehavior footLoadBearingBehavior = new FootLoadBearingBehavior("Bloppy", ros2Node);

      FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(RobotSide.LEFT, LoadBearingRequest.LOAD);

      PacketDestination destination = PacketDestination.UI;
      message.setDestination(destination.ordinal());

      footLoadBearingBehavior.setInput(message);

      assertTrue("Input was not set correctly.", footLoadBearingBehavior.hasInputBeenSet());
   }
}
