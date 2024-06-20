package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
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

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class NotStartedState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final YoEnum<ContinuousHikingState> continuousHikingState;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;

   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public NotStartedState(ROS2Node ros2Node,
                          String simpleRobotName,
                          HumanoidReferenceFrames referenceFrames,
                          YoEnum<ContinuousHikingState> continuousHikingState,
                          AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                          ContinuousPlanner continuousPlanner,
                          ContinuousHikingParameters continuousHikingParameters,
                          TerrainMapData terrainMap,
                          TerrainPlanningDebugger debugger)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingState = continuousHikingState;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;

      pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
   }

   @Override
   public void onEntry()
   {
      LogTools.warn(continuousHikingState.getEnumValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         continuousHikingState.set(ContinuousHikingState.NOT_STARTED);

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
            continuousHikingState.set(ContinuousHikingState.PLAN_AVAILABLE);
         }
         else
         {
            continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
            continuousPlanner.setInitialized(false);

            String message = "";
            LogTools.error(message = String.format("State: [%s]: Initialization failed... will retry initializing next tick",
                                                   continuousHikingState.getEnumValue()));
            statistics.appendString(message);
         }
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
         continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
   }
}
