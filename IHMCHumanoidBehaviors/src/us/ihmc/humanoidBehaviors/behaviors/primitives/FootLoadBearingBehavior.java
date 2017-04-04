package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class FootLoadBearingBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private FootLoadBearingMessage outgoingFootLoadBearingMessage;

   public FootLoadBearingBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }
   public FootLoadBearingBehavior(String prefix, CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(prefix,outgoingCommunicationBridge);
   }

   public void setInput(FootLoadBearingMessage endEffectorLoadBearingMessage)
   {
      this.outgoingFootLoadBearingMessage = endEffectorLoadBearingMessage;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingFootLoadBearingMessage != null))
      {
         sendFootStateToController();
      }
   }

   private void sendFootStateToController()
   {
      if (!isPaused.getBooleanValue() &&!isAborted.getBooleanValue())
      {
         sendPacketToController(outgoingFootLoadBearingMessage);
         packetHasBeenSent.set(true);
      }
   }


   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      outgoingFootLoadBearingMessage = null;

      isPaused.set(false);
      isAborted.set(false);
   }


   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() &&!isPaused.getBooleanValue();
   }

   
   
   public boolean hasInputBeenSet() {
	   if (outgoingFootLoadBearingMessage != null)
		   return true;
	   else
		   return false;
   }
   @Override
   public void onBehaviorEntered()
   {
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
