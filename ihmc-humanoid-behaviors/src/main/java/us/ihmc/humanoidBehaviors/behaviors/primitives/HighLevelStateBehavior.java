package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.yoVariables.variable.YoBoolean;

public class HighLevelStateBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private HighLevelStateMessage outgoingHighLevelStatePacket;

   public HighLevelStateBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public void setInput(HighLevelStateMessage highLevelStatePacket)
   {
      this.outgoingHighLevelStatePacket = highLevelStatePacket;
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
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         sendPacketToController(outgoingHighLevelStatePacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);
      
      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      outgoingHighLevelStatePacket = null;

      isPaused.set(false);
      isAborted.set(false);
   }


   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
   }


   public boolean hasInputBeenSet()
   {
      if (outgoingHighLevelStatePacket != null)
         return true;
      else
         return false;
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
