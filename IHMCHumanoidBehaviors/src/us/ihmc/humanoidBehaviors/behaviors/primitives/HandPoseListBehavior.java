package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.tools.FormattingTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseListBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final ConcurrentListeningQueue<HandPoseStatus> inputListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();
   private Status status;

   private HandPoseListPacket outgoingHandPoseListPacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;

   private final BooleanYoVariable hasInputBeenSet;

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
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);
      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      hasStatusBeenReceived = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);
      
      this.attachControllerListeningQueue(inputListeningQueue, HandPoseStatus.class);
   }

   public void setInput(HandPoseListPacket handPoseListPacket)
   {
      this.outgoingHandPoseListPacket = handPoseListPacket;
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());
      
      if (!hasPacketBeenSent.getBooleanValue() && outgoingHandPoseListPacket != null)
      {
         sendHandPoseListToController();
      }
      
      if (inputListeningQueue.isNewPacketAvailable())
      {
         consumeHandPoseStatus(inputListeningQueue.getNewestPacket());
      }
      
      if (!isDone.getBooleanValue() && status == Status.COMPLETED && hasInputBeenSet() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
      {
         if (DEBUG)
            PrintTools.debug(this, outgoingHandPoseListPacket.getRobotSide() + " HandPoseListBehavior setting isDone = true");
         isDone.set(true);
      }
   }
   
   private void consumeHandPoseStatus(HandPoseStatus handPoseStatus)
   {
      if ((handPoseStatus != null) && (handPoseStatus.getRobotSide() == outgoingHandPoseListPacket.getRobotSide()))
      {
         if (DEBUG)
            PrintTools.debug(this, "Received a hand pose status: " + handPoseStatus.getStatus() + ", " + handPoseStatus.getRobotSide() + " at t = " + yoTime.getDoubleValue());
         status = handPoseStatus.getStatus();
         hasStatusBeenReceived.set(true);
      }
   }

   public Status getStatus()
   {
      return status;
   }

   private void sendHandPoseListToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingHandPoseListPacket.setDestination(PacketDestination.CONTROLLER);

         if (DEBUG)
            PrintTools.debug(this, "sending handPoseList packet to controller and network processor: " + outgoingHandPoseListPacket);
         
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
      hasPacketBeenSent.set(false);
      outgoingHandPoseListPacket = null;

      isPaused.set(false);
      isStopped.set(false);

      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      startTime.set(Double.NaN);
      status = null;
      isDone.set(false);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      hasPacketBeenSent.set(false);
      outgoingHandPoseListPacket = null;

      isPaused.set(false);
      isStopped.set(false);

      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
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
         sendHandPoseListToController();
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
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
