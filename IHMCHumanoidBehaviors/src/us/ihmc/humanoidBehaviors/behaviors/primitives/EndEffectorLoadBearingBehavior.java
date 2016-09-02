package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class EndEffectorLoadBearingBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private EndEffectorLoadBearingMessage outgoingEndEffectorLoadBearingMessage;

   public EndEffectorLoadBearingBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
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
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingEndEffectorLoadBearingMessage);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);
      outgoingEndEffectorLoadBearingMessage = null;

      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() &&!isPaused.getBooleanValue();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }
   
   public boolean hasInputBeenSet() {
	   if (outgoingEndEffectorLoadBearingMessage != null)
		   return true;
	   else
		   return false;
   }
}
