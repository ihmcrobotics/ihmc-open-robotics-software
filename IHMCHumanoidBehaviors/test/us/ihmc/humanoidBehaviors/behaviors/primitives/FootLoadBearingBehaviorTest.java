package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootLoadBearingBehaviorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetInput()
   {
      CommunicationBridgeInterface outgoingCommunicationBridge = null;
      FootLoadBearingBehavior footLoadBearingBehavior = new FootLoadBearingBehavior(outgoingCommunicationBridge);

      FootLoadBearingMessage message = new FootLoadBearingMessage(RobotSide.LEFT, LoadBearingRequest.LOAD);

      PacketDestination destination = PacketDestination.UI;
      message.setDestination(destination);

      footLoadBearingBehavior.setInput(message);

      assertTrue("Input was not set correctly.", footLoadBearingBehavior.hasInputBeenSet());
   }
}
