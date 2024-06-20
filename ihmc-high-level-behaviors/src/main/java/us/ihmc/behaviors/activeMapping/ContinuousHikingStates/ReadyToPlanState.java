package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class ReadyToPlanState implements State
{
   private final YoEnum<ContinuousHikingState> continuousHikingState;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;

   private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();
   private List<QueuedFootstepStatusMessage> controllerQueue;
   private int controllerQueueSize = 0;
   private String message = "";

   public ReadyToPlanState(ROS2Helper ros2Helper,
                           String simpleRobotName,
                           YoEnum<ContinuousHikingState> continuousHikingState,
                           AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ContinuousHikingParameters continuousHikingParameters,
                           TerrainMapData terrainMap,
                           TerrainPlanningDebugger debugger)
   {
      this.continuousHikingState = continuousHikingState;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                               continuousHikingState.getEnumValue()));
         statistics.appendString(message);
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   /*
    * Callback to receive a message each time there is a change in the FootstepStatusMessage
    */
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      this.latestFootstepStatusMessage.set(footstepStatusMessage);
   }

   @Override
   public void onEntry()
   {
      LogTools.warn(continuousHikingState.getEnumValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      statistics.setLastAndTotalWaitingTimes();

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         continuousPlanner.getImminentStanceFromLatestStatus(latestFootstepStatusMessage, controllerQueue);
      }

      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      continuousPlanner.setGoalWaypointPoses();
      continuousPlanner.planToGoal(commandMessage.get());
      continuousPlanner.logFootStePlan();

      if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get()
                                                                                                                                .getUseMonteCarloPlanAsReference())
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
         // TODO: Add replanning. Replanned plan valid only until the foot lands.
         continuousHikingState.set(ContinuousHikingState.WAITING_TO_LAND);
         LogTools.error(message = String.format("State: [%s]: Planning failed... will try again when current step is completed",
                                                continuousHikingState.getEnumValue()));
         statistics.appendString(message);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
         continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
   }
}
