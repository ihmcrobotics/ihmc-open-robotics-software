package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class NotStartedState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final StepValidityChecker stepValidityChecker;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;

   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   private int controllerQueueSize = 0;

   public NotStartedState(ROS2Helper ros2Helper,
                          String simpleRobotName,
                          HumanoidReferenceFrames referenceFrames,
                          AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                          StepValidityChecker stepValidityChecker,
                          ContinuousPlanner continuousPlanner,
                          ContinuousHikingParameters continuousHikingParameters,
                          TerrainMapData terrainMap,
                          TerrainPlanningDebugger debugger)
   {
      this.referenceFrames = referenceFrames;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);

      pauseWalkingPublisher = ros2Helper.getROS2NodeInterface().createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
      this.stepValidityChecker = stepValidityChecker;
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
         LogTools.warn(message );
         statistics.appendString(message);
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }


   @Override
   public void onEntry()
   {
      LogTools.warn("Entering [NOT_STATED] state");
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         PauseWalkingMessage message = new PauseWalkingMessage();

         if (continuousPlanner.isInitialized())
         {
            message.setPause(true);
            message.setClearRemainingFootstepQueue(true);
            pauseWalkingPublisher.publish(message);
         }

         RobotSide closerSide = continuousPlanner.getCloserSideToGoal();
         FramePose3D closerToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide));
         FramePose3D fartherToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide.getOppositeSide()));
         closerToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         fartherToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         if (continuousPlanner.updateImminentStance(fartherToGoalFootPose, closerToGoalFootPose, closerSide))
         {
            debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
         }

         continuousPlanner.setInitialized(false);
         continuousPlanner.requestMonteCarloPlannerReset();

         return;
      }

      if (!continuousPlanner.isInitialized())
      {
         continuousPlanner.initialize();
         continuousPlanner.setGoalWaypointPoses();
         continuousPlanner.planToGoal(commandMessage.get());
         if (commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get().getUseMonteCarloPlanAsReference() || commandMessage.get()
                                                                                                                                               .getUseHybridPlanner())
         {
            debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
            debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
         }

         if (continuousPlanner.isPlanAvailable())
         {
            stepValidityChecker.checkNextStepIsValid(continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters, controllerQueue));
         }
         else
         {
            continuousPlanner.setInitialized(false);

            String message = String.format("State: [%s]: Initialization failed... will retry initializing next tick");
            LogTools.error(message);
            statistics.appendString(message);
         }
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return continuousPlanner.isPlanAvailable();
   }
}
