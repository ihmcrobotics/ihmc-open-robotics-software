package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.BumStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class BumStateBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private BumStatePacket outgoingBumStatePacket;

   public BumStateBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public void setInput(BumStatePacket bumStatePacket)
   {
      this.outgoingBumStatePacket = bumStatePacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingBumStatePacket != null))
      {
         sendBumStatePacketToController();
      }
   }

   private void sendBumStatePacketToController()
   {
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingBumStatePacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);
      
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);
      outgoingBumStatePacket = null;

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
   
   @Override
   public boolean hasInputBeenSet() {
	   if (outgoingBumStatePacket != null)
		   return true;
	   else
		   return false;
   }
}
