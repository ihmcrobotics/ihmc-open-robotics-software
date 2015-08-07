package us.ihmc.humanoidBehaviors.behaviors.primitives;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandStatePacket;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandStateBehaviorTest
{
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 300000)
   public void testSetInput()
   {
      OutgoingCommunicationBridgeInterface outgoingCommunicationBridge = null;
      DoubleYoVariable yoTimeDummy = null;
      HandStateBehavior handStateBehavior = new HandStateBehavior(outgoingCommunicationBridge, yoTimeDummy);
      
      HandStatePacket handStatePacket = new HandStatePacket(RobotSide.LEFT, true, true);
      
      PacketDestination destination = PacketDestination.UI;
      handStatePacket.setDestination(destination);
      
      handStateBehavior.setInput(handStatePacket);
      
      assertTrue("Input was not set correctly.", handStateBehavior.hasInputBeenSet());
   }
}
