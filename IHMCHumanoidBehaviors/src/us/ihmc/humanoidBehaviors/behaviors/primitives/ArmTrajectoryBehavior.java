package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class ArmTrajectoryBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final ConcurrentListeningQueue<HandPoseStatus> inputListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();

   protected RobotSide robotSide;
   protected Status status;

   protected ArmTrajectoryMessage outgoingMessage;

   protected final BooleanYoVariable hasPacketBeenSent;
   protected final DoubleYoVariable yoTime;
   protected final DoubleYoVariable startTime;
   protected final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;

   protected final BooleanYoVariable hasInputBeenSet;
   protected final BooleanYoVariable hasStatusBeenReceived;
   private final BooleanYoVariable isDone;

   public ArmTrajectoryBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public ArmTrajectoryBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
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

   public void setInput(ArmTrajectoryMessage armTrajectoryMessage)
   {
      outgoingMessage = armTrajectoryMessage;

      robotSide = armTrajectoryMessage.getRobotSide();
      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(armTrajectoryMessage.getTrajectoryTime());

      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());

      if (inputListeningQueue.isNewPacketAvailable())
      {
         consumeHandPoseStatus(inputListeningQueue.getNewestPacket());
      }

      if (!isDone.getBooleanValue() && status == Status.COMPLETED && hasInputBeenSet() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
      {
         if (DEBUG)
            PrintTools.debug(this, robotSide + " ArmTrajectoryBehavior setting isDone = true");
         isDone.set(true);
      }

      if (!hasPacketBeenSent.getBooleanValue() && (outgoingMessage != null))
      {
         sendOutgoingPacketToControllerAndNetworkProcessor();
      }
   }

   public Status getStatus()
   {
      return status;
   }

   private void sendOutgoingPacketToControllerAndNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingMessage.setDestination(PacketDestination.UI);

         sendPacketToController(outgoingMessage);
         sendPacketToNetworkProcessor(outgoingMessage);

         hasPacketBeenSent.set(true);

         if (DEBUG)
            PrintTools.debug(this, "sending packet to controller and network processor: " + outgoingMessage);
      }
   }

   private void stopArmMotion()
   {
      if (outgoingMessage != null)
      {
         StopAllTrajectoryMessage pausePacket = new StopAllTrajectoryMessage();
         pausePacket.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(pausePacket);
      }
   }

   @Override
   public void initialize()
   {
      inputListeningQueue.clear();
      status = null;
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      hasStatusBeenReceived.set(false);
      isPaused.set(false);
      isDone.set(false);
      hasBeenInitialized.set(true);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      isPaused.set(false);
      isStopped.set(false);

      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);

      status = null;
      isDone.set(false);
   }

   @Override
   public void stop()
   {
      stopArmMotion();
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      if (isPaused.getBooleanValue())
      {
         return;
      }
      else
      {
         stopArmMotion();
         status = null;
         isPaused.set(true);
      }
   }

   @Override
   public void resume()
   {
      if (!isPaused.getBooleanValue())
      {
         return;
      }
      else
      {
         status = null;
         isPaused.set(false);

         if (hasInputBeenSet())
         {
            sendOutgoingPacketToControllerAndNetworkProcessor();
         }
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private void consumeHandPoseStatus(HandPoseStatus handPoseStatus)
   {
      RobotSide statusRobotSide = handPoseStatus.getRobotSide();

      if (statusRobotSide == robotSide)
      {
         status = handPoseStatus.getStatus();
         hasStatusBeenReceived.set(true);
         if (DEBUG)
            PrintTools.debug(this,
                  "Received a hand pose status: " + handPoseStatus.getStatus() + ", " + handPoseStatus.getRobotSide() + " at t = " + yoTime.getDoubleValue());
      }
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
