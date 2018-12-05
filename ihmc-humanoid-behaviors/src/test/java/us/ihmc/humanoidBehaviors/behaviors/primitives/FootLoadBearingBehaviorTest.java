package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;

public class FootLoadBearingBehaviorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSetInput()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "test_set_input");
      FootLoadBearingBehavior footLoadBearingBehavior = new FootLoadBearingBehavior("Bloppy", ros2Node);

      FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(RobotSide.LEFT, LoadBearingRequest.LOAD);

      PacketDestination destination = PacketDestination.UI;
      message.setDestination(destination.ordinal());

      footLoadBearingBehavior.setInput(message);

      assertTrue("Input was not set correctly.", footLoadBearingBehavior.hasInputBeenSet());
   }
}
