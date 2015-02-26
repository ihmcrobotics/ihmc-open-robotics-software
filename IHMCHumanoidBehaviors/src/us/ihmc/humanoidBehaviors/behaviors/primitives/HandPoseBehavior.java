package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseBehavior extends BehaviorInterface
{
   private final boolean DEBUG = false;

   private final ConcurrentListeningQueue<HandPoseStatus> inputListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();
   private final ConcurrentListeningQueue<HandCollisionDetectedPacket> collisionListeningQueue = new ConcurrentListeningQueue<HandCollisionDetectedPacket>();
   private Status status;

   private HandPosePacket outgoingHandPosePacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;

   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable hasStatusBeenReceived;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable stopHandIfCollision;

   public HandPoseBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public HandPoseBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "trajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);
      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      hasStatusBeenReceived = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);
      stopHandIfCollision = new BooleanYoVariable(behaviorNameFirstLowerCase + "StopIfCollision", registry);
      this.attachControllerListeningQueue(inputListeningQueue, HandPoseStatus.class);
      this.attachControllerListeningQueue(collisionListeningQueue, HandCollisionDetectedPacket.class);
   }

   public void setInput(Frame frame, RigidBodyTransform pose, RobotSide robotSide, double trajectoryTime)
   {
      setInput(frame, pose, robotSide, trajectoryTime, false);
   }

   public void setInput(Frame frame, RigidBodyTransform pose, RobotSide robotSide, double trajectoryTime, boolean stopHandIfCollision)
   {
      setInput(PacketControllerTools.createHandPosePacket(frame, pose, robotSide, trajectoryTime), stopHandIfCollision);
   }

   public void setInput(HandPosePacket handPosePacket)
   {
      setInput(handPosePacket, false);
   }

   public void setInput(HandPosePacket handPosePacket, boolean stopHandIfCollision)
   {
      this.stopHandIfCollision.set(stopHandIfCollision);
      this.outgoingHandPosePacket = handPosePacket;
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
            SysoutTool.println(outgoingHandPosePacket.getRobotSide() + " HandPoseBehavior setting isDone = true");
         isDone.set(true);
      }
      
      if (stopHandIfCollision.getBooleanValue() && !isStopped.getBooleanValue() && collisionListeningQueue.isNewPacketAvailable() && trajectoryTimeElapsed.getDoubleValue() > 0.1)
      {
         SysoutTool.println("COLLISION DETECTED!  STOPPING HAND.");
         stop();
         isDone.set(true);
      }

      
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingHandPosePacket != null))
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
         outgoingHandPosePacket.setDestination(PacketDestination.UI);

         if (DEBUG)
            SysoutTool.println("sending handPose packet to controller and network processor: " + outgoingHandPosePacket);
         sendPacketToController(outgoingHandPosePacket);
         sendPacketToNetworkProcessor(outgoingHandPosePacket);

         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHandPosePacket.getTrajectoryTime());
      }
   }

   private void stopArmMotion()
   {
      if (outgoingHandPosePacket != null)
      {
         RobotSide robotSide = outgoingHandPosePacket.getRobotSide();
         if (robotSide != null)
         {
            StopArmMotionPacket pausePacket = new StopArmMotionPacket(robotSide);
            pausePacket.setDestination(PacketDestination.CONTROLLER);
            sendPacketToController(pausePacket);
         }
      }
   }

   @Override
   public void initialize()
   {
      if (hasInputBeenSet())
      {
         SysoutTool.println("WARNING:  INITIALIZING BEHAVIOR *AFTER* INPUT HAS BEEN SET!");
      }
      inputListeningQueue.clear();
      status = null;
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingHandPosePacket = null;

      hasStatusBeenReceived.set(false);
      isPaused.set(false);
      isDone.set(false);
      hasBeenInitialized.set(true);
   }

   @Override
   public void finalize()
   {
      hasPacketBeenSent.set(false);
      outgoingHandPosePacket = null;

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
            sendHandPoseToController();
         }
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();

      // if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
      // trajectoryTimeElapsed.set(false);
      // else
      // trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());
      //
      // return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   private void consumeHandPoseStatus(HandPoseStatus handPoseStatus)
   {
      if ((handPoseStatus != null) && (handPoseStatus.getRobotSide() == outgoingHandPosePacket.getRobotSide()))
      {
         if (DEBUG)
            SysoutTool.println("Received a hand pose status: " + handPoseStatus.getStatus() + ", " + handPoseStatus.getRobotSide());
         status = handPoseStatus.getStatus();
         hasStatusBeenReceived.set(true);
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
