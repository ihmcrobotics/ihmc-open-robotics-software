package us.ihmc.humanoidRobotics.communication;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<PlanOffsetStatus> planOffsetMessage = new AtomicReference<>(new PlanOffsetStatus());

   private boolean footstepStarted;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   private final AtomicBoolean robotFalling = new AtomicBoolean(false);

   public ControllerFootstepQueueMonitor(ROS2Node ros2Node, String simpleRobotName)
   {
      ros2Node.createSubscription2(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      ros2Node.createSubscription2(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      ros2Node.createSubscription2(getTopic(PlanOffsetStatus.class, simpleRobotName), this::acceptPlanOffsetStatus);
      ros2Node.createSubscription2(getTopic(WalkingStatusMessage.class, simpleRobotName), this::acceptWalkingStatusMessage);
      ros2Node.createSubscription2(getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), this::acceptWalkingControllerFailureStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         String message = String.format("Latest Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
         LogTools.info(message);
      }

      // For the statistics set the that controller queue size before getting the new one
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      footstepStarted = footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED;

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void acceptWalkingStatusMessage(WalkingStatusMessage message)
   {
      // Declared locally since this represents the absolute state which other threads can access
      isWalking.set(false);
      WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());

      if (walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED)
      {
         isWalking.set(true);
      }
   }

   private void acceptPlanOffsetStatus(PlanOffsetStatus planOffsetMessage)
   {
      this.planOffsetMessage.set(planOffsetMessage);
   }

   public List<QueuedFootstepStatusMessage> getControllerFootstepQueue()
   {
      return controllerQueue;
   }

   public AtomicReference<FootstepStatusMessage> getFootstepStatusMessage()
   {
      return footstepStatusMessage;
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return controllerQueueSize;
   }

   /**
    * This method assumes the list is not empty; you need to check outside this method that the list has at least one in it
    */
   public FramePose3DReadOnly getLastFootstepInQueue()
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      previousFootstepPose.getPosition().set(controllerQueue.get(controllerQueueSize - 1).getLocation());
      previousFootstepPose.getRotation().setToYawOrientation(controllerQueue.get(controllerQueueSize - 1).getOrientation().getYaw());

      return previousFootstepPose;
   }

   /**
    * This method assumes the list is not empty; you need to check outside this method that the list has at least one in it
    */
   public FramePose3DReadOnly getLastFootstepQueuedOnOppositeSide(RobotSide candidateFootstepSide)
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      int i = controllerQueue.size() - 1;
      // Moved the index of the list to the last step on the other side
      while (i >= 1 && controllerQueue.get(i).getRobotSide() == candidateFootstepSide.toByte())
         --i;

      previousFootstepPose.getPosition().set(controllerQueue.get(i).getLocation());
      previousFootstepPose.getRotation().setToYawOrientation(controllerQueue.get(i).getOrientation().getYaw());

      return previousFootstepPose;
   }

   private void acceptWalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage message)
   {
      robotFalling.set(true);
   }

   public boolean pollRobotFalling()
   {
      return robotFalling.getAndSet(false);
   }

   // TODO Polling this in multiple threads may cause issues as the second time its pulled the value will be null.
   // TODO If needed this should change to handle that case correctly.
   public PlanOffsetStatus pollPlanOffsetMessage()
   {
      return planOffsetMessage.getAndSet(null);
   }

   public boolean isFootstepStarted()
   {
      return footstepStarted;
   }

   // TODO Polling this in multiple threads may cause issues as the second time its pulled the value will be null.
   // TODO If needed this should change to handle that case correctly.
   public boolean pollIsWalking()
   {
      return isWalking.getAndSet(false);
   }
}
