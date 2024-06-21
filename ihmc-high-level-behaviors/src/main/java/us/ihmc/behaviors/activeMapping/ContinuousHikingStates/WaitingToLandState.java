package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class WaitingToLandState implements State
{
   private final ROS2Helper ros2Helper;
   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousHikingParameters continuousHikingParameters;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final ContinuousPlanner continuousPlanner;

   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();

   public WaitingToLandState(ROS2Helper ros2Helper,
                             String simpleRobotName,
                             HumanoidReferenceFrames referenceFrames,
                             ContinuousPlanner continuousPlanner,
                             ContinuousHikingParameters continuousHikingParameters)
   {
      this.ros2Helper = ros2Helper;
      this.referenceFrames = referenceFrames;
      this.continuousHikingParameters = continuousHikingParameters;
      this.continuousPlanner = continuousPlanner;

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      ros2Helper.createPublisher(controllerFootstepDataTopic);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      // Set the that controller queue size before getting the new one
      statistics.setLastFootstepQueueLength(controllerQueueSize);

      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         String message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
         LogTools.warn(message);
         statistics.appendString(message);
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   /*
    * Callback to receive a message each time there is a change in the FootstepStatusMessage
    */
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         // TODO: Use the transfer time (starting now) to start planning (if WAITING_TO_LAND then plan again)
         statistics.incrementTotalStepsCompleted();

         double distance = referenceFrames.getSoleFrame(RobotSide.LEFT)
                                          .getTransformToDesiredFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT))
                                          .getTranslation()
                                          .norm();
         statistics.setLastLengthCompleted((float) distance);

         statistics.logToFile(true, true);
      }

      this.latestFootstepStatusMessage.set(footstepStatusMessage);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      FootstepStatusMessage footstepStatusMessage = latestFootstepStatusMessage.get();
      return footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED;
   }

   @Override
   public void onEntry()
   {
      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         if (continuousPlanner.isPlanAvailable())
         {
            FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters, controllerQueue);

            String message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
            LogTools.info(message);
            statistics.appendString(message);

            ros2Helper.publish(controllerFootstepDataTopic, footstepDataList);
         }
         else
         {
            String message = String.format("State: [%s]: Planning failed... will try again when current step is completed");
            LogTools.error(message);
            statistics.appendString(message);
         }
      }

      continuousPlanner.setPlanAvailable(false);
      continuousPlanner.transitionCallback();
      statistics.setStartWaitingTime();


      LogTools.warn("Entering [WAITING_TO_LAND] state");
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit(double timeInState)
   {
      statistics.endStepTime();
      statistics.startStepTime();
   }
}
