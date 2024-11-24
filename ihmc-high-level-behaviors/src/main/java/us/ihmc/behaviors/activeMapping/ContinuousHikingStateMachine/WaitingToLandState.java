package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingLogger;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.atomic.AtomicReference;

public class WaitingToLandState implements State
{
   private final ROS2Helper ros2Helper;
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;
   private final ContinuousHikingParameters continuousHikingParameters;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private FootstepStatusMessage previousFootstepStatusMessage = null;
   private final ContinuousHikingLogger continuousHikingLogger;
   private boolean isDone = false;

   /**
    * This state exists to send a plan to the controller (that is if we have a plan to send). Then we will exist this state when the next step is started. Once
    * that step is started we want to plan again in order to keep walking, so leave this state.
    */
   public WaitingToLandState(ROS2Helper ros2Helper,
                             String simpleRobotName,
                             AtomicReference<ContinuousHikingCommandMessage> commandMessage,
                             ContinuousPlanner continuousPlanner,
                             ControllerFootstepQueueMonitor controllerQueueMonitor,
                             ContinuousHikingParameters continuousHikingParameters,
                             ContinuousHikingLogger continuousHikingLogger)
   {
      this.ros2Helper = ros2Helper;
      this.commandMessage = commandMessage;
      this.continuousHikingParameters = continuousHikingParameters;
      this.continuousPlanner = continuousPlanner;
      this.controllerQueueMonitor = controllerQueueMonitor;
      this.continuousHikingLogger = continuousHikingLogger;

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      ros2Helper.createPublisher(controllerFootstepDataTopic);
   }

   @Override
   public void onEntry()
   {
      isDone = false;

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         if (continuousPlanner.isPlanAvailable())
         {
            FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters,
                                                                                                           controllerQueueMonitor.getControllerFootstepQueue());

            String message = String.format("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
            LogTools.info(message);
            continuousHikingLogger.appendString(message);

            ros2Helper.publish(controllerFootstepDataTopic, footstepDataList);
            continuousPlanner.setPlanAvailable(false);
         }
         else
         {
            continuousPlanner.setLatestFootstepPlan(null);
            String message = "State: Planning failed... will try again when current step is completed";
            LogTools.error(message);
            continuousHikingLogger.appendString(message);
         }
      }

      continuousPlanner.transitionCallback();
   }

   @Override
   public void doAction(double timeInState)
   {
      if (controllerQueueMonitor.getControllerFootstepQueue() != null && controllerQueueMonitor.getControllerFootstepQueue().isEmpty() && !continuousPlanner.isInitialized() && !continuousPlanner.isPlanAvailable())
      {
         isDone = true;
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (isDone)
      {
         return true;
      }

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
