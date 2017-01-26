package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class EndEffectorLoadBearingBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private EndEffectorLoadBearingMessage outgoingEndEffectorLoadBearingMessage;

   public EndEffectorLoadBearingBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }
   public EndEffectorLoadBearingBehavior(String prefix, CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(prefix,outgoingCommunicationBridge);
   }

   public void setInput(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      this.outgoingEndEffectorLoadBearingMessage = endEffectorLoadBearingMessage;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingEndEffectorLoadBearingMessage != null))
      {
         sendFootStateToController();
      }
   }

   private void sendFootStateToController()
   {
      if (!isPaused.getBooleanValue() &&!isAborted.getBooleanValue())
      {
         sendPacketToController(outgoingEndEffectorLoadBearingMessage);
         packetHasBeenSent.set(true);
      }
   }


   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      outgoingEndEffectorLoadBearingMessage = null;

      isPaused.set(false);
      isAborted.set(false);
   }


   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() &&!isPaused.getBooleanValue();
   }

   
   
   public boolean hasInputBeenSet() {
	   if (outgoingEndEffectorLoadBearingMessage != null)
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
