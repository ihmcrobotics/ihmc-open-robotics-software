package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerStatistics;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Topic;

public class WaitingToLandState implements State
{
   private final ROS2Helper ros2Helper;
   private final ContinuousHikingParameters continuousHikingParameters;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private FootstepStatusMessage previousFootstepStatusMessage = null;
   private final ContinuousPlannerStatistics statistics;

   public WaitingToLandState(ROS2Helper ros2Helper,
                             String simpleRobotName,
                             ContinuousPlanner continuousPlanner,
                             ControllerFootstepQueueMonitor controllerQueueMonitor,
                             ContinuousHikingParameters continuousHikingParameters,
                             ContinuousPlannerStatistics statistics)
   {
      this.ros2Helper = ros2Helper;
      this.continuousHikingParameters = continuousHikingParameters;
      this.continuousPlanner = continuousPlanner;
      this.controllerQueueMonitor = controllerQueueMonitor;
      this.statistics = statistics;

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      ros2Helper.createPublisher(controllerFootstepDataTopic);
   }

   @Override
   public void onEntry()
   {
      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         if (continuousPlanner.isPlanAvailable())
         {
            FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters,
                                                                                                           controllerQueueMonitor.getControllerFootstepQueue());

            String message = String.format("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
            LogTools.info(message);
            statistics.appendString(message);

            ros2Helper.publish(controllerFootstepDataTopic, footstepDataList);
            continuousPlanner.setPlanAvailable(false);
         }
         else
         {
            continuousPlanner.setLatestFootstepPlan(null);
            String message = "State: Planning failed... will try again when current step is completed";
            LogTools.error(message);
            statistics.appendString(message);
         }
      }

      continuousPlanner.transitionCallback();
      statistics.setStartWaitingTime();
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

   @Override
   public boolean isDone(double timeInState)
   {
      //TODO this is a bit messy, cleanup please
      if (controllerQueueMonitor.getFootstepStatusMessage() != null)
      {
         FootstepStatusMessage footstepStatusMessage = controllerQueueMonitor.getFootstepStatusMessage().get();
         if (previousFootstepStatusMessage != null && previousFootstepStatusMessage.getSequenceId() == footstepStatusMessage.getSequenceId())
            return false;

         previousFootstepStatusMessage = footstepStatusMessage;
         return footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED;
      }

      return false;
   }
}
