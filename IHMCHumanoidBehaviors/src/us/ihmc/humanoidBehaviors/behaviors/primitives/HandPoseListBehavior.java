package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseListBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private Status status;

   private HandPoseListPacket outgoingHandPoseListPacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;

   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable trajectoryTimeElapsed;

   private final BooleanYoVariable hasStatusBeenReceived;

   private final BooleanYoVariable isDone;

   public HandPoseListBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public HandPoseListBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      trajectoryTimeElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      hasStatusBeenReceived = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);
   }

   public void setInput(HandPoseListPacket handPoseListPacket)
   {
      this.outgoingHandPoseListPacket = handPoseListPacket;
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && outgoingHandPoseListPacket != null)
      {
         sendHandPoseToController();
      }
   }

   public Status getStatus()
   {
      return status;
   }

   private void sendHandPoseToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingHandPoseListPacket.setDestination(PacketDestination.UI);

         sendPacketToController(outgoingHandPoseListPacket);
         sendPacketToNetworkProcessor(outgoingHandPoseListPacket);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHandPoseListPacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      status = null;
      trajectoryTimeElapsed.set(false);
      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);
      isPaused.set(false);
      isDone.set(false);
   }

   @Override
   public void finalize()
   {
      hasPacketBeenSent.set(false);
      outgoingHandPoseListPacket = null;

      isPaused.set(false);
      isStopped.set(false);

      trajectoryTimeElapsed.set(false);
      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      status = null;
      isDone.set(false);
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
      hasPacketBeenSent.set(false);
      if (hasInputBeenSet())
      {
         sendHandPoseToController();
      }
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
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
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
