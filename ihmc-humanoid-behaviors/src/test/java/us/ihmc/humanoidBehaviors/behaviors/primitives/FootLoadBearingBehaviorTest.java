package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import static us.ihmc.robotics.Assert.assertTrue;

public class FootLoadBearingBehaviorTest
{
   @Test
   public void testSetInput()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("test_set_input");
      FootLoadBearingBehavior footLoadBearingBehavior = new FootLoadBearingBehavior("Bloppy", ros2Node);

      FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(RobotSide.LEFT, LoadBearingRequest.LOAD);

      PacketDestination destination = PacketDestination.UI;
      message.setDestination(destination.ordinal());

      footLoadBearingBehavior.setInput(message);

      assertTrue("Input was not set correctly.", footLoadBearingBehavior.hasInputBeenSet());
   }
}
