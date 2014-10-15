package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
  private HandPosePacket outgoingHandPosePacket;

   private final DoubleYoVariable yoTime;
   private double startTime = Double.NaN;
   private double currentTime = Double.NaN;
   private double behaviorTime = 1.0;

   public HandPoseBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
   }

   public void setInput(HandPosePacket handPosePacket)
   {
      this.outgoingHandPosePacket = handPosePacket;
      behaviorTime = handPosePacket.getTrajectoryTime();
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && outgoingHandPosePacket != null)
      {
         sendHandPoseToController();
      }

      currentTime = yoTime.getDoubleValue();
   }

   private void sendHandPoseToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingHandPosePacket);
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
      outgoingHandPosePacket = null;

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
      return trajectoryTimeElapsed && !isPaused.getBooleanValue();
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
	   if (outgoingHandPosePacket != null)
		   return true;
	   else
		   return false;
   }
}
