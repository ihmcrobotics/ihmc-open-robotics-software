package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerStatistics;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.log.LogTools;
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
   private final StepValidityChecker stepValidityChecker;

   public ReadyToPlanState(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ContinuousHikingParameters continuousHikingParameters,
                           TerrainMapData terrainMap,
                           TerrainPlanningDebugger debugger,
                           ContinuousPlannerStatistics statistics,
                           StepValidityChecker stepValidityChecker)
   {
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;
      this.statistics = statistics;
      this.stepValidityChecker = stepValidityChecker;
   }

   @Override
   public void onEntry()
   {
      LogTools.warn(String.format("Entering %s", getClass().getSimpleName()));

      continuousPlanner.initialize();
      continuousPlanner.setPlanAvailable(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      statistics.setLastAndTotalWaitingTimes();

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         if (controllerFootstepQueueMonitor.getFootstepStatusMessage() != null)
            continuousPlanner.getImminentStanceFromLatestStatus(controllerFootstepQueueMonitor.getFootstepStatusMessage(),
                                                                controllerFootstepQueueMonitor.getControllerFootstepQueue());
      }

      continuousPlanner.setGoalWaypointPoses();
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
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
         stepValidityChecker.checkNextStepIsValid(continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters,
                                                                                                      controllerFootstepQueueMonitor.getControllerFootstepQueue()));

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
      return continuousPlanner.isPlanAvailable() && continuousHikingParameters.getStepPublisherEnabled();
   }
}
