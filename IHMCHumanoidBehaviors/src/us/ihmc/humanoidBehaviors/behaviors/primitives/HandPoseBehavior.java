package us.ihmc.humanoidBehaviors.behaviors.primitives;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final ConcurrentListeningQueue<HandPoseStatus> inputListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();
   private Status status;

   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private HandPosePacket outgoingHandPosePacket;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;

   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable trajectoryTimeElapsed;

   private final BooleanYoVariable hasStatusBeenSent;

   private final BooleanYoVariable isDone;

   public HandPoseBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      startTime = new DoubleYoVariable(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      hasInputBeenSet = new BooleanYoVariable(getName() + "HasInputBeenSet", registry);
      trajectoryTimeElapsed = new BooleanYoVariable(getName() + "TrajectoryTimeElapsed", registry);
      hasStatusBeenSent = new BooleanYoVariable(getName() + "HasStatusBeenSent", registry);
      isDone = new BooleanYoVariable(getName() + "IsDone", registry);
      this.attachControllerListeningQueue(inputListeningQueue, HandPoseStatus.class);
   }

   public void setInput(HandPosePacket handPosePacket)
   {
      this.outgoingHandPosePacket = handPosePacket;
      hasInputBeenSet.set(true);
   }

   public void goToHomePosition(RobotSide side)
   {
      setInput(HandPosePacket.createGoToHomePacket(side, 3.0));
   }

   public void setInput(Frame frame, RigidBodyTransform pose, RobotSide robotSide, double trajectoryTime)
   {
      Vector3d translation = new Vector3d();
      Quat4d rotation = new Quat4d();
      pose.get(translation);
      pose.get(rotation);
      Point3d point = new Point3d(translation.getX(), translation.getY(), translation.getZ());
      setInput(new HandPosePacket(robotSide, frame, point, rotation, trajectoryTime));
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && outgoingHandPosePacket != null)
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

         sendPacketToController(outgoingHandPosePacket);
         sendPacketToNetworkProcessor(outgoingHandPosePacket);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHandPosePacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      status = null;
      trajectoryTimeElapsed.set(false);
      hasInputBeenSet.set(false);
      hasStatusBeenSent.set(false);
      isPaused.set(false);
      isDone.set(false);
   }

   @Override
   public void finalize()
   {
      packetHasBeenSent.set(false);
      outgoingHandPosePacket = null;

      isPaused.set(false);
      isStopped.set(false);

      trajectoryTimeElapsed.set(false);
      hasInputBeenSet.set(false);
      hasStatusBeenSent.set(false);

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
      StopArmMotionPacket pausePacket = new StopArmMotionPacket(outgoingHandPosePacket.getRobotSide());
      pausePacket.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(pausePacket);
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
      packetHasBeenSent.set(false);
      if (hasInputBeenSet())
      {
         sendHandPoseToController();
      }
   }

   @Override
   public boolean isDone()
   {
      checkForHandPoseStatus();
      if (status == Status.COMPLETED && !hasStatusBeenSent.getBooleanValue())
      {
         hasStatusBeenSent.set(true);
         isDone.set(true);
      }
      else
      {
         isDone.set(false);
      }
      return isDone.getBooleanValue();
      //      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
      //         trajectoryTimeElapsed.set(false);
      //      else
      //         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());
      //
      //      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   private void checkForHandPoseStatus()
   {
      HandPoseStatus newestPacket = inputListeningQueue.getNewestPacket();
      if (newestPacket != null && newestPacket.getRobotSide() == outgoingHandPosePacket.getRobotSide())
      {
         if (DEBUG)
            System.out.println("Received a hand pose status: " + newestPacket.getStatus() + ", " + newestPacket.getRobotSide());
         status = newestPacket.getStatus();
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
