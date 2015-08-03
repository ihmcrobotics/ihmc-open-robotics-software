package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.ThighStatePacket;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.robotSide.RobotSide;

public class ThighStateBehaviorTest
{
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 300000)
   public void testSetInput()
   {
      OutgoingCommunicationBridgeInterface outgoingCommunicationBridge = null;
      ThighStateBehavior thighStateBehavior = new ThighStateBehavior(outgoingCommunicationBridge);
      
      ThighStatePacket thighStatePacket = new ThighStatePacket(RobotSide.LEFT, true);
      
      PacketDestination destination = PacketDestination.UI;
      thighStatePacket.setDestination(destination);
      
      thighStateBehavior.setInput(thighStatePacket);
      
      assertTrue("Input was not set correctly.", thighStateBehavior.hasInputBeenSet());
   }
}
