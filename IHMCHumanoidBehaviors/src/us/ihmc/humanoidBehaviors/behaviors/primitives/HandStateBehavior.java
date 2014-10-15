package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.manipulation.HandStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandStateBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private HandStatePacket outgoingHandStatePacket;

   private final DoubleYoVariable yoTime;
   private double startTime = Double.NaN;
   private double currentTime = Double.NaN;

   // TODO: This is just an estimate of the time out. This needs to be dealt with
   private double behaviorTime = 4.0;

   public HandStateBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
   }

   public void setInput(HandStatePacket handStatePacket)
   {
      this.outgoingHandStatePacket = handStatePacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingHandStatePacket != null))
      {
         sendHandPoseToController();
      }

      currentTime = yoTime.getDoubleValue();
   }

   private void sendHandPoseToController()
   {
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingHandStatePacket);
         packetHasBeenSent.set(true);
         startTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void finalize()
   {
      packetHasBeenSent.set(false);
      outgoingHandStatePacket = null;

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
      boolean trajectoryTimeElapsed = currentTime - startTime > behaviorTime;

      return trajectoryTimeElapsed &&!isPaused.getBooleanValue();
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
	   if (outgoingHandStatePacket != null)
		   return true;
	   else
		   return false;
   }
}
