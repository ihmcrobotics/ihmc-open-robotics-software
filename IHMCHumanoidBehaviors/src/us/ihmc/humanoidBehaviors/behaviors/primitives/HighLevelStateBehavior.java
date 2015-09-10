package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class HighLevelStateBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private HighLevelStatePacket outgoingHighLevelStatePacket;

   public HighLevelStateBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public void setInput(HighLevelStatePacket thighStatePacket)
   {
      this.outgoingHighLevelStatePacket = thighStatePacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingHighLevelStatePacket != null))
      {
         sendPacketToController();
      }
   }

   private void sendPacketToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingHighLevelStatePacket);
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
      outgoingHighLevelStatePacket = null;

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
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
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

   public boolean hasInputBeenSet()
   {
      if (outgoingHighLevelStatePacket != null)
         return true;
      else
         return false;
   }
}
