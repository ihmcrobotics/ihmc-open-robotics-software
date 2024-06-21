package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class WaitingToLandState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final YoEnum<ContinuousHikingState> continuousHikingState;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousHikingParameters continuousHikingParameters;

   private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();

   public WaitingToLandState(ROS2Helper ros2Helper,
                             String simpleRobotName,
                             HumanoidReferenceFrames referenceFrames,
                             YoEnum<ContinuousHikingState> continuousHikingState,
                             AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                             ContinuousHikingParameters continuousHikingParameters)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingState = continuousHikingState;
      this.commandMessage = commandMessage;
      this.continuousHikingParameters = continuousHikingParameters;

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
   }

   /*
    * Callback to receive a message each time there is a change in the FootstepStatusMessage
    */
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         continuousHikingState.set(ContinuousHikingState.READY_TO_PLAN);
         statistics.endStepTime();
         statistics.startStepTime();
      }
      else if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
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
   public void onEntry()
   {
      LogTools.warn("Entering [WAITING_TO_LAND] state");
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
         continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
   }
}
