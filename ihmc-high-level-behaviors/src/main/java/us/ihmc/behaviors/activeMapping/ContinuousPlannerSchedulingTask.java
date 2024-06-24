package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingStatesAndTransitions.*;
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

   public static ContinuousPlannerStatistics statistics = new ContinuousPlannerStatistics();

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

      continuousPlanner.setContinuousPlannerStatistics(statistics);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      StateMachineFactory<ContinuousHikingState, State> stateMachineFactory = new StateMachineFactory<>(ContinuousHikingState.class);
      stateMachineFactory.setNamePrefix("ContinuousHikingStateMachine");
      stateMachineFactory.setRegistry(registry);

      StepValidityChecker stepValidityChecker = new StepValidityChecker(continuousPlanner, robotModel, referenceFrames, registry);
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Helper, simpleRobotName, referenceFrames);

      State notStartedState = new DoNothingState(ros2Helper,
                                                 simpleRobotName,
                                                 referenceFrames,
                                                 continuousPlanner,
                                                 debugger);

      State readyToPlanState = new ReadyToPlanState(commandMessage,
                                                    continuousPlanner,
                                                    controllerFootstepQueueMonitor,
                                                    continuousHikingParameters,
                                                    terrainMap,
                                                    debugger,
                                                    stepValidityChecker);

      State waitingtoLandState = new WaitingToLandState(ros2Helper,
                                                        simpleRobotName,
                                                        referenceFrames,
                                                        continuousPlanner,
                                                        controllerFootstepQueueMonitor,
                                                        continuousHikingParameters,
                                                        debugger);

      // Adding the different states
      stateMachineFactory.addState(ContinuousHikingState.DO_NOTHING, notStartedState);
      stateMachineFactory.addState(ContinuousHikingState.WAITING_TO_LAND, waitingtoLandState);
      stateMachineFactory.addState(ContinuousHikingState.READY_TO_PLAN, readyToPlanState);

      StartContinuousHikingTransitionCondition startContinuousHikingTransitionCondition = new StartContinuousHikingTransitionCondition(commandMessage,
                                                                                                                                       continuousHikingParameters);

      stateMachineFactory.addTransition(ContinuousHikingState.DO_NOTHING, ContinuousHikingState.READY_TO_PLAN, startContinuousHikingTransitionCondition);

      StopContinuousHikingTransitionCondition stopContinuousHikingTransitionCondition = new StopContinuousHikingTransitionCondition(continuousHikingParameters,
                                                                                                                                    commandMessage);
      //TODO is matters what orders these are added in, if the first condition is true it won't do the rest

      // From any given state we can go back to DO_NOTHING and stop ContinuousHiking
      stateMachineFactory.addTransition(ContinuousHikingState.WAITING_TO_LAND, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);


      PlanAgainTransitionCondition planAgainTransitionCondition = new PlanAgainTransitionCondition(continuousPlanner, continuousHikingParameters);

      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.READY_TO_PLAN, planAgainTransitionCondition);

//      stateMachineFactory.addTransition(ContinuousHikingState.DO_NOTHING, ContinuousHikingState.WAITING_TO_LAND, startingStepTransitionCondition);
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

      // When the state machine does a transition, it will set continuousHikingState to null, that way when it goes into the next state.
      // It doesn't have any transition that needs to happen yet until we set one in order to go to the next state
      // TODO fix statistic prints because the enum gets reset to null when going into a new state, so those prints are useless
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