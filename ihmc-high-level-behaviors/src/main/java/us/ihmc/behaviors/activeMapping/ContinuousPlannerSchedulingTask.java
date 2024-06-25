package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is responsible for scheduling the continuous planner state machine. It is responsible for handling the state transitions and the logic of the
 * state machine.
 */
public class ContinuousPlannerSchedulingTask
{
   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final TerrainPlanningDebugger debugger;
   private TerrainMapData terrainMap;

   private final ContinuousPlanner continuousPlanner;

   public StateMachine<ContinuousHikingState, State> stateMachine;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousHikingParameters continuousHikingParameters,
                                          ContinuousPlanner.PlanningMode mode)
   {
      String simpleRobotName = robotModel.getSimpleRobotName();

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND, commandMessage::set);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLACED_GOAL_FOOTSTEPS, this::addWayPointCheckPointToList);

      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      debugger = new TerrainPlanningDebugger(ros2Node, monteCarloPlannerParameters);
      this.continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, mode, continuousHikingParameters, monteCarloPlannerParameters, debugger);

      ContinuousPlannerStatistics statistics = new ContinuousPlannerStatistics();
      continuousPlanner.setContinuousPlannerStatistics(statistics);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      StateMachineFactory<ContinuousHikingState, State> stateMachineFactory = new StateMachineFactory<>(ContinuousHikingState.class);
      stateMachineFactory.setNamePrefix("ContinuousHikingStateMachine");
      stateMachineFactory.setRegistry(registry);

      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Helper,
                                                                                                         simpleRobotName,
                                                                                                         referenceFrames,
                                                                                                         statistics);

      // Create the different states
      State notStartedState = new DoNothingState(ros2Helper, simpleRobotName, referenceFrames, continuousPlanner);
      State readyToPlanState = new ReadyToPlanState(commandMessage,
                                                    continuousPlanner,
                                                    controllerFootstepQueueMonitor,
                                                    continuousHikingParameters,
                                                    terrainMap,
                                                    debugger,
                                                    statistics);
      State waitingtoLandState = new WaitingToLandState(ros2Helper,
                                                        simpleRobotName,
                                                        continuousPlanner,
                                                        controllerFootstepQueueMonitor,
                                                        continuousHikingParameters,
                                                        statistics);

      // Adding the different states
      stateMachineFactory.addState(ContinuousHikingState.DO_NOTHING, notStartedState);
      stateMachineFactory.addState(ContinuousHikingState.WAITING_TO_LAND, waitingtoLandState);
      stateMachineFactory.addState(ContinuousHikingState.READY_TO_PLAN, readyToPlanState);

      // Create different conditions
      StartContinuousHikingTransitionCondition startContinuousHikingTransitionCondition = new StartContinuousHikingTransitionCondition(commandMessage,
                                                                                                                                       continuousHikingParameters);
      StopContinuousHikingTransitionCondition stopContinuousHikingTransitionCondition = new StopContinuousHikingTransitionCondition(continuousHikingParameters,
                                                                                                                                    commandMessage);
      PlanAgainTransitionCondition planAgainTransitionCondition = new PlanAgainTransitionCondition(continuousPlanner, continuousHikingParameters);

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

      stateMachine = stateMachineFactory.build(ContinuousHikingState.DO_NOTHING);
      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousHikingState
    */
   private void tickStateMachine()
   {
      continuousPlanner.syncParametersCallback();

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

   public ContinuousPlanner getContinuousPlanner()
   {
      return continuousPlanner;
   }

   public void destroy()
   {
      executorService.shutdown();
   }

   public void addWayPointCheckPointToList(PoseListMessage poseListMessage)
   {
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      continuousPlanner.addWayPointToList(poses.get(0), poses.get(1));
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
   }
}