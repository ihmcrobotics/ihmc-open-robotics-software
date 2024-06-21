package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
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
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;

   private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();

   public ReadyToPlanState(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ContinuousHikingParameters continuousHikingParameters,
                           TerrainMapData terrainMap,
                           TerrainPlanningDebugger debugger)
   {
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;

      controllerFootstepQueueMonitor.attachFootstepStatusMessageConsumer(latestFootstepStatusMessage::set);
   }

   private boolean didPlanningLoop;

   @Override
   public void onEntry()
   {
      LogTools.warn("Entering [READY_TO_PLAN] state");
      didPlanningLoop = false;
   }

   @Override
   public void doAction(double timeInState)
   {
      statistics.setLastAndTotalWaitingTimes();

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         continuousPlanner.getImminentStanceFromLatestStatus(latestFootstepStatusMessage, controllerFootstepQueueMonitor.getControllerFootstepQueue());
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

      didPlanningLoop = true;
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return didPlanningLoop;
   }

}
