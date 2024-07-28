package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine.*;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is responsible for scheduling the continuous hiking state machine. It is responsible for handling the state transitions and the logic of the
 * state machine.
 */
public class ContinuousPlannerSchedulingTask
{
   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   public enum PlanningMode
   {
      FAST_HIKING, WALK_TO_GOAL
   }

   // The default mode for when things start up
   private PlanningMode planningMode = PlanningMode.FAST_HIKING;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final ContinuousPlanner continuousPlanner;
   public StateMachine<ContinuousHikingState, State> stateMachine;
   private TerrainMapData terrainMap;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousHikingParameters continuousHikingParameters,
                                          MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters,
                                          DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                                          SwingPlannerParametersBasics swingPlannerParameters)
   {
      String simpleRobotName = robotModel.getSimpleRobotName();

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND, commandMessage::set);

      TerrainPlanningDebugger debugger = new TerrainPlanningDebugger(ros2Node, monteCarloFootstepPlannerParameters, planningMode);
      ContinuousPlannerStatistics statistics = new ContinuousPlannerStatistics();
      continuousPlanner = new ContinuousPlanner(robotModel,
                                                referenceFrames,
                                                continuousHikingParameters,
                                                monteCarloFootstepPlannerParameters,
                                                footstepPlannerParameters,
                                                swingPlannerParameters,
                                                debugger,
                                                statistics);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      StateMachineFactory<ContinuousHikingState, State> stateMachineFactory = new StateMachineFactory<>(ContinuousHikingState.class);
      stateMachineFactory.setNamePrefix("ContinuousHikingStateMachine");
      stateMachineFactory.setRegistry(registry);

      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Helper,
                                                                                                         simpleRobotName,
                                                                                                         referenceFrames,
                                                                                                         statistics);

      // Create the different states
      State notStartedState = new DoNothingState(ros2Helper, simpleRobotName, continuousPlanner, debugger);
      State readyToPlanState = new ReadyToPlanState(ros2Helper,
                                                    referenceFrames,
                                                    commandMessage,
                                                    continuousPlanner,
                                                    controllerFootstepQueueMonitor,
                                                    continuousHikingParameters,
                                                    terrainMap,
                                                    debugger,
                                                    statistics,
                                                    planningMode);
      State waitingtoLandState = new WaitingToLandState(ros2Helper,
                                                        simpleRobotName,
                                                        continuousPlanner,
                                                        controllerFootstepQueueMonitor,
                                                        continuousHikingParameters,
                                                        statistics);

      // Adding the different states
      stateMachineFactory.addState(ContinuousHikingState.DO_NOTHING, notStartedState);
      stateMachineFactory.addState(ContinuousHikingState.READY_TO_PLAN, readyToPlanState);
      stateMachineFactory.addState(ContinuousHikingState.WAITING_TO_LAND, waitingtoLandState);

      // Create different conditions
      StartContinuousHikingTransitionCondition startContinuousHikingTransitionCondition = new StartContinuousHikingTransitionCondition(commandMessage,
                                                                                                                                       continuousHikingParameters);
      StopContinuousHikingTransitionCondition stopContinuousHikingTransitionCondition = new StopContinuousHikingTransitionCondition(commandMessage,
                                                                                                                                    continuousHikingParameters);
      PlanAgainTransitionCondition planAgainTransitionCondition = new PlanAgainTransitionCondition(continuousPlanner, continuousHikingParameters);

      //NOTE: The transitions for the state machine are checked in order they are added. And once one condition is true the other's don't get checked.
      // In order to be able to always stop the state machine we add the stop conditions first

      // From any given state we can go back to DO_NOTHING and stop ContinuousHiking
      stateMachineFactory.addTransition(ContinuousHikingState.WAITING_TO_LAND, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);

      // Add condition, this triggers the state machine to start Continuous Hiking
      stateMachineFactory.addTransition(ContinuousHikingState.DO_NOTHING, ContinuousHikingState.READY_TO_PLAN, startContinuousHikingTransitionCondition);

      // Add condition, this allows us to plan over and over again without sending any footsteps to the controller
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.READY_TO_PLAN, planAgainTransitionCondition);

      // Add done conditions in order to go into the next state
      stateMachineFactory.addDoneTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.WAITING_TO_LAND);
      stateMachineFactory.addDoneTransition(ContinuousHikingState.WAITING_TO_LAND, ContinuousHikingState.READY_TO_PLAN);

      // Added a couple listeners to help when jumping between states
      stateMachine = stateMachineFactory.build(ContinuousHikingState.DO_NOTHING);
      stateMachineFactory.addStateChangedListener((from, to) -> LogTools.warn("STATE CHANGED: ( " + from + " -> " + to + " )"));
      stateMachineFactory.addStateChangedListener((from, to) -> planningMode = debugger.getPlanningMode());

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous hiking state machine every {@link #CONTINUOUS_PLANNING_DELAY_MS} milliseconds. The state is stored in the
    * {@link ContinuousHikingState}
    */
   private void tickStateMachine()
   {
      stateMachine.doActionAndTransition();
   }

   public void setLatestHeightMapData(HeightMapData heightMapData)
   {
      this.continuousPlanner.setLatestHeightMapData(heightMapData);
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMap = new TerrainMapData(terrainMapData);
      this.continuousPlanner.setLatestTerrainMapData(terrainMapData);
   }

   public void destroy()
   {
      executorService.shutdown();
   }
}