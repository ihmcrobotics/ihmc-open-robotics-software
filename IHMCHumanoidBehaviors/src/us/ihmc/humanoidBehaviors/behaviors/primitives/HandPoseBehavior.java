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
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class HandPoseBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final ConcurrentListeningQueue<HandPoseStatus> inputListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();
   private final ConcurrentListeningQueue<HandCollisionDetectedPacket> collisionListeningQueue = new ConcurrentListeningQueue<HandCollisionDetectedPacket>();
   private Status status;

   private HandPosePacket outgoingHandPosePacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;
   private final DoubleYoVariable percentTimeRemainingAtCollision;

   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable hasStatusBeenReceived;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable stopHandIfCollision;
   private final BooleanYoVariable isPausedDueToCollision;
   private final IntegerYoVariable numberOfConsecutiveCollisions;
   private final DoubleYoVariable yoCollisionTime;

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
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);
      percentTimeRemainingAtCollision = new DoubleYoVariable(behaviorNameFirstLowerCase + "PercentTimeRemainingAtCollision", registry);
      percentTimeRemainingAtCollision.set(Double.NaN);

      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      hasStatusBeenReceived = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);
      stopHandIfCollision = new BooleanYoVariable(behaviorNameFirstLowerCase + "StopIfCollision", registry);
      isPausedDueToCollision = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsPausedDueToCollision", registry);
      numberOfConsecutiveCollisions = new IntegerYoVariable(behaviorNameFirstLowerCase + "ConsecutiveCollisionCount", registry);
      yoCollisionTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "CollisionTime", registry);
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
            PrintTools.debug(this, outgoingHandPosePacket.getRobotSide() + " HandPoseBehavior setting isDone = true");
         isDone.set(true);
         numberOfConsecutiveCollisions.set(0);
      }

      if (collisionListeningQueue.isNewPacketAvailable())
      {
         if (stopHandIfCollision.getBooleanValue() && !isStopped.getBooleanValue() && !isPaused.getBooleanValue()
               && trajectoryTimeElapsed.getDoubleValue() > 0.5)
            handleHandCollision(collisionListeningQueue.getNewestPacket());
      }

      if (isPausedDueToCollision.getBooleanValue())
      {
         if (retryHandPose && yoTime.getDoubleValue() > yoCollisionTime.getDoubleValue() + 0.5)
         {
            //            HandPosePacket gentlerVersionOfPreviousPacket = PacketControllerTools.createHandPosePacket(outgoingHandPosePacket.referenceFrame,
            //                  outgoingHandPosePacket.position, outgoingHandPosePacket.orientation, outgoingHandPosePacket.robotSide,
            //                  2.0 * outgoingHandPosePacket.trajectoryTime);

            resume();
            isPausedDueToCollision.set(false);
            PrintTools.debug(this, "Re-Attemping HandPoseBehavior");
         }
      }

      if (!hasPacketBeenSent.getBooleanValue() && (outgoingHandPosePacket != null))
      {
         sendHandPosePacketToController(outgoingHandPosePacket);
      }
   }

   private boolean retryHandPose;

   public void handleHandCollision(HandCollisionDetectedPacket collisionPacket)
   {
      int collisionSeverityOneToThree = collisionPacket.getCollisionSeverityOneToThree();

      yoCollisionTime.set(yoTime.getDoubleValue());

      if (numberOfConsecutiveCollisions.getIntegerValue() == 0)
         percentTimeRemainingAtCollision.set(100.0 * (trajectoryTime.getDoubleValue() - trajectoryTimeElapsed.getDoubleValue())
               / trajectoryTime.getDoubleValue());

      if (DEBUG)
      {
         PrintTools.debug(this, "COLLISION DETECTED! Severity [1-3]: " + collisionSeverityOneToThree);
         PrintTools.debug(this, "PercentTimeRemainingAtInitialCollision: " + percentTimeRemainingAtCollision.getDoubleValue());
         PrintTools.debug(this, "Number of consecutive collisions: " + numberOfConsecutiveCollisions.getIntegerValue());
      }

      //TODO: Reduce number of retries if collision severity *increases*
      int maxNumberOfTimesToRetryHandPose = 10;

      // reduce number of retries as hand gets closer to target
      retryHandPose = numberOfConsecutiveCollisions.getIntegerValue() < (int) Math.round(maxNumberOfTimesToRetryHandPose
            * percentTimeRemainingAtCollision.getDoubleValue() / 100.0);

      if (retryHandPose)
      {
         pause();
         isPausedDueToCollision.set(true);
      }
      else
      {
         isPausedDueToCollision.set(false);
         stopArmMotion();
         finalize();
         isDone.set(true);
         PrintTools.debug(this, "Giving Up on Re-Attempting HandPoseBehavior \n");
      }

      numberOfConsecutiveCollisions.add(1);
   }

   public Status getStatus()
   {
      return status;
   }

   private void sendHandPosePacketToController(HandPosePacket handPosePacket)
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         handPosePacket.setDestination(PacketDestination.UI);

         if (DEBUG)
            PrintTools.debug(this, "sending handPose packet to controller and network processor: " + handPosePacket);
         sendPacketToController(handPosePacket);
         sendPacketToNetworkProcessor(handPosePacket);

         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(handPosePacket.getTrajectoryTime());
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
         PrintTools.debug(this, "WARNING:  INITIALIZING BEHAVIOR *AFTER* INPUT HAS BEEN SET!");
      }
      inputListeningQueue.clear();
      status = null;
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingHandPosePacket = null;

      hasStatusBeenReceived.set(false);
      isPaused.set(false);
      isPausedDueToCollision.set(false);
      isDone.set(false);
      hasBeenInitialized.set(true);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      percentTimeRemainingAtCollision.set(Double.NaN);
      numberOfConsecutiveCollisions.set(0);
      yoCollisionTime.set(Double.NaN);
   }

   @Override
   public void finalize()
   {
      hasPacketBeenSent.set(false);
      outgoingHandPosePacket = null;

      isPaused.set(false);
      isPausedDueToCollision.set(false);
      isStopped.set(false);

      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      percentTimeRemainingAtCollision.set(Double.NaN);
      numberOfConsecutiveCollisions.set(0);
      yoCollisionTime.set(Double.NaN);

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
            sendHandPosePacketToController(outgoingHandPosePacket);
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
      if ((handPoseStatus != null) && (handPoseStatus.getRobotSide() == outgoingHandPosePacket.getRobotSide()))
      {
         if (DEBUG)
            PrintTools.debug(this, "Received a hand pose status: " + handPoseStatus.getStatus() + ", " + handPoseStatus.getRobotSide() + " at t = " + yoTime.getDoubleValue());
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
