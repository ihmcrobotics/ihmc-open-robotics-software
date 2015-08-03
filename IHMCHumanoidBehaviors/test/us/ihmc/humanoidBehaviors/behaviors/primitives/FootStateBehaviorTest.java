package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.FootStatePacket;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootStateBehaviorTest
{
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 300000)
   public void testSetInput()
   {
      OutgoingCommunicationBridgeInterface outgoingCommunicationBridge = null;
      FootStateBehavior footStateBehavior = new FootStateBehavior(outgoingCommunicationBridge);
      
      FootStatePacket footStatePacket = new FootStatePacket(RobotSide.LEFT, true);
      
      PacketDestination destination = PacketDestination.UI;
      footStatePacket.setDestination(destination);
      
      footStateBehavior.setInput(footStatePacket);
      
      assertTrue("Input was not set correctly.", footStateBehavior.hasInputBeenSet());
   }
}
