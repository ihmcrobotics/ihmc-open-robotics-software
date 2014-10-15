package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ComHeightBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private ComHeightPacket outgoingComHeightPacket;

   private final DoubleYoVariable yoTime;
   private double startTime = Double.NaN;
   private double currentTime = Double.NaN;

   // TODO: This is just an estimate of the time out. This needs to be dealt with
   private double behaviorTime = 4.0;

   public ComHeightBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
   }

   public void setInput(ComHeightPacket comHeightPacket)
   {
      this.outgoingComHeightPacket = comHeightPacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingComHeightPacket != null))
      {
         sendHandPoseToController();
      }

      currentTime = yoTime.getDoubleValue();
   }

   private void sendHandPoseToController()
   {
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingComHeightPacket);
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
      outgoingComHeightPacket = null;

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

   @Override
   public boolean hasInputBeenSet() {
	   if (outgoingComHeightPacket != null)
		   return true;
	   else
		   return false;
   }
}
