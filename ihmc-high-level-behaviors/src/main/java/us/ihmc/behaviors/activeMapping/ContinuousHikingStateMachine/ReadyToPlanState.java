package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import org.apache.commons.lang.time.StopWatch;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerStatistics;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicReference;

public class ReadyToPlanState implements State
{
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;
   private final ContinuousPlannerStatistics statistics;

   private final StopWatch stopWatch = new StopWatch();
   double timeInSwingToStopPlanningAndWaitTillNextAttempt = 0;

   public ReadyToPlanState(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ContinuousHikingParameters continuousHikingParameters,
                           TerrainMapData terrainMap,
                           TerrainPlanningDebugger debugger,
                           ContinuousPlannerStatistics statistics)
   {
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;
      this.statistics = statistics;
   }

   @Override
   public void onEntry()
   {
      continuousPlanner.setPlanAvailable(false);
      stopWatch.reset();
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      timeInSwingToStopPlanningAndWaitTillNextAttempt = continuousHikingParameters.getSwingTime() * continuousHikingParameters.getPercentThroughSwingToPlanTo();
      stopWatch.start();
   }

   @Override
   public void doAction(double timeInState)
   {
      statistics.setLastAndTotalWaitingTimes();

      // These may be null if no steps have been sent to the controller, good to check that here
      if (controllerFootstepQueueMonitor.getFootstepStatusMessage() != null && controllerFootstepQueueMonitor.getControllerFootstepQueue() != null)
      {
         continuousPlanner.setLatestFootstepStatusMessage(controllerFootstepQueueMonitor.getFootstepStatusMessage());
         continuousPlanner.setLatestControllerQueue(controllerFootstepQueueMonitor.getControllerFootstepQueue());
      }

      // Set up the imminent stance and goal poses in which to plan from
      continuousPlanner.setImminentStanceToPlanFrom();
      continuousPlanner.setGoalWaypointPoses();
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());

      // Plan to the goal and log the plan
      continuousPlanner.planToGoal(commandMessage.get());
      continuousPlanner.logFootStePlan();

      if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get()
                                                                                                                                .getUseMonteCarloPlanAsReference())
      {
         debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
         debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
      }

      // We know that we have a plan, and that only gets set to true when we have at least one step in the plan, so we know it's not empty
      if (continuousPlanner.isPlanAvailable())
      {
         FootstepDataListMessage message = FootstepDataMessageConverter.createFootstepDataListFromPlan(continuousPlanner.getLatestFootstepPlan(),
                                                                                                       continuousHikingParameters.getSwingTime(),
                                                                                                       continuousHikingParameters.getTransferTime());
         debugger.publishPlannedFootsteps(message);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      boolean stopPlanningAndCompleteCurrentStep = stopWatch.getTime() > timeInSwingToStopPlanningAndWaitTillNextAttempt;
      return stopPlanningAndCompleteCurrentStep || (continuousPlanner.isPlanAvailable() && continuousHikingParameters.getStepPublisherEnabled());
   }
}
