package us.ihmc.humanoidRobotics.communication;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepQueueStatusMessage;
import controller_msgs.FootstepStatusMessage;
import controller_msgs.PlanOffsetStatus;
import controller_msgs.QueuedFootstepStatusMessage;
import controller_msgs.WalkingControllerFailureStatusMessage;
import controller_msgs.WalkingStatusMessage;
import ihmc_common_msgs.QueueableMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.HumanoidControllerAPI.getLowFrequencyTopic;
import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private IDLObjectSequence<QueuedFootstepStatusMessage> controllerQueue;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<PlanOffsetStatus> planOffsetMessage = new AtomicReference<>(new PlanOffsetStatus());

   private boolean footstepStarted;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   private final AtomicBoolean isWalkingPaused = new AtomicBoolean(false);
   private final AtomicBoolean robotFalling = new AtomicBoolean(false);
   private final AtomicBoolean receivedNewFootstepPlanWithOverride = new AtomicBoolean(false);

   public ControllerFootstepQueueMonitor(ROS2Node ros2Node, String simpleRobotName)
   {
      ros2Node.createSubscription(getLowFrequencyTopic(FootstepQueueStatusMessage.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.footstepQueueStatusReceived(message);
      });
      ros2Node.createSubscription(getTopic(FootstepStatusMessage.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.footstepStatusReceived(message);
      });
      ros2Node.createSubscription(getLowFrequencyTopic(PlanOffsetStatus.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.acceptPlanOffsetStatus(message);
      });
      ros2Node.createSubscription(getTopic(WalkingStatusMessage.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.acceptWalkingStatusMessage(message);
      });
      ros2Node.createSubscription(getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.acceptWalkingControllerFailureStatusMessage(message);
      });
      ros2Node.createSubscription(getTopic(FootstepDataListMessage.class, simpleRobotName), reader -> {
         var message = reader.read();
         if (message != null)
            this.interceptFootstepDataListMessage(message);
      });
   }

   private void interceptFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      if (footstepDataListMessage.getOffsetFootstepsHeightWithExecutionError())
      {
         if (footstepDataListMessage.getQueueingProperties().getExecutionMode() == QueueableMessage.EXECUTION_MODE_OVERRIDE)
         {
            receivedNewFootstepPlanWithOverride.set(true);
         }
      }
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

      if (walkingStatus == WalkingStatus.STARTED)
      {
         isWalking.set(true);
         isWalkingPaused.set(false);
      }
      else if (walkingStatus == WalkingStatus.PAUSED)
      {
         isWalkingPaused.set(true);
      }
      else if (walkingStatus == WalkingStatus.RESUMED)
      {
         isWalkingPaused.set(false);
      }
   }

   private void acceptPlanOffsetStatus(PlanOffsetStatus planOffsetMessage)
   {
      this.planOffsetMessage.set(planOffsetMessage);
   }

   public IDLObjectSequence<QueuedFootstepStatusMessage> getControllerFootstepQueue()
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
   public FramePose3DReadOnly getFirstFootstepInQueue()
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      previousFootstepPose.getPosition().set(controllerQueue.get(0).getLocation().getPoint());
      previousFootstepPose.getRotation().setToYawOrientation(controllerQueue.get(0).getOrientation().getQuaternion().getYaw());

      return previousFootstepPose;
   }

   /**
    * This method assumes the list is not empty; you need to check outside this method that the list has at least one in it
    */
   public FramePose3DReadOnly getLastFootstepInQueue()
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      previousFootstepPose.getPosition().set(controllerQueue.get(controllerQueueSize - 1).getLocation().getPoint());
      previousFootstepPose.getRotation().setToYawOrientation(controllerQueue.get(controllerQueueSize - 1).getOrientation().getQuaternion().getYaw());

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

      previousFootstepPose.getPosition().set(controllerQueue.get(i).getLocation().getPoint());
      previousFootstepPose.getRotation().setToYawOrientation(controllerQueue.get(i).getOrientation().getQuaternion().getYaw());

      return previousFootstepPose;
   }

   public boolean getReceivedNewFootstepPlanWithOverride()
   {
      return receivedNewFootstepPlanWithOverride.getAndSet(false);
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

   public boolean pollIsWalkingPaused()
   {
      return isWalkingPaused.get();
   }
}
