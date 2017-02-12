package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;

public class EndEffectorLoadBearingBehaviorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetInput()
   {
      CommunicationBridgeInterface outgoingCommunicationBridge = null;
      EndEffectorLoadBearingBehavior endEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior(outgoingCommunicationBridge);

      EndEffectorLoadBearingMessage message = new EndEffectorLoadBearingMessage(RobotSide.LEFT, EndEffector.FOOT, LoadBearingRequest.LOAD);
      
      PacketDestination destination = PacketDestination.UI;
      message.setDestination(destination);
      
      endEffectorLoadBearingBehavior.setInput(message);
      
      assertTrue("Input was not set correctly.", endEffectorLoadBearingBehavior.hasInputBeenSet());
   }
}
