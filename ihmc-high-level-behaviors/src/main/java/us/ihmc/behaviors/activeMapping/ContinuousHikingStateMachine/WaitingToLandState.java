package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingLogger;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

public class WaitingToLandState implements State
{
   private final ContinuousHikingParameters continuousHikingParameters;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final ROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private FootstepStatusMessage previousFootstepStatusMessage = new FootstepStatusMessage();
   private final ContinuousHikingLogger continuousHikingLogger;

   /**
    * This state exists to send a plan to the controller (that is if we have a plan to send). Then we will exist this state when the next step is started. Once
    * that step is started we want to plan again in order to keep walking, so leave this state.
    */
   public WaitingToLandState(ROS2Node ros2Node,
                             String simpleRobotName,
                             ContinuousPlanner continuousPlanner,
                             ControllerFootstepQueueMonitor controllerQueueMonitor,
                             ContinuousHikingParameters continuousHikingParameters,
                             ContinuousHikingLogger continuousHikingLogger)
   {
      this.continuousHikingParameters = continuousHikingParameters;
      this.continuousPlanner = continuousPlanner;
      this.controllerQueueMonitor = controllerQueueMonitor;
      this.continuousHikingLogger = continuousHikingLogger;

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      footstepPublisher = ros2Node.createPublisher(controllerFootstepDataTopic);
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
            continuousHikingLogger.appendString(message);

            footstepPublisher.publish(footstepDataList);
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
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      FootstepStatusMessage footstepStatusMessage = controllerQueueMonitor.getFootstepStatusMessage().get();
      if (previousFootstepStatusMessage.getSequenceId() == footstepStatusMessage.getSequenceId())
         return false;

      previousFootstepStatusMessage = footstepStatusMessage;
      return footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED;
   }
}
