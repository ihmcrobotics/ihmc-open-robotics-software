package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

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
    * This limits the number of steps in a session, will cause the state machine to stop
    */
   private final static int MAX_STEPS_PER_SESSION = 100;

   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 50;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
   private final TerrainPlanningDebugger debugger;
   private TerrainMapData terrainMap;

   private final ContinuousPlannerStatistics statistics;
   private final ContinuousHikingParameters parameters;
   private final FootstepPoseHeuristicChecker stepChecker;

   private final ContinuousPlanner continuousPlanner;
   private final HumanoidReferenceFrames referenceFrames;

   private String message = "";
   private int stepCounter = 0;
   private int controllerQueueSize = 0;

   public StateMachine<ContinuousHikingState, State> stateMachine;
   public YoEnum<ContinuousHikingState> continuousHikingMode;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousHikingParameters parameters,
                                          ContinuousPlanner.PlanningMode mode)
   {
      this.referenceFrames = referenceFrames;
      this.parameters = parameters;
      String simpleRobotName = robotModel.getSimpleRobotName();

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND, commandMessage::set);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLACED_GOAL_FOOTSTEPS, this::addWayPointCheckPointToList);

      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      debugger = new TerrainPlanningDebugger(ros2Node, monteCarloPlannerParameters);
      this.continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, mode, parameters, monteCarloPlannerParameters, debugger);

      statistics = new ContinuousPlannerStatistics();
      continuousPlanner.setContinuousPlannerStatistics(statistics);

      // FIXME this needs to get a copy of the height map or terrain map for the step checker to actually work as intended.
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(FootstepPlanningModuleLauncher.createFootPolygons(robotModel),
                                                                  continuousPlanner.getFootstepPlannerParameters(),
                                                                  environmentHandler);
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      stepChecker = new FootstepPoseHeuristicChecker(continuousPlanner.getFootstepPlannerParameters(), snapper, registry);

      StateMachineFactory<ContinuousHikingState, State> factory = new StateMachineFactory<>(ContinuousHikingState.class);
      factory.setNamePrefix("ContinuousHikingStateMachine");
      factory.setRegistry(registry);

      State notStartedState = new NotStartedState(ros2Node, simpleRobotName);
      State readyToPlanState = new ReadyToPlanState(ros2Helper, simpleRobotName);
      State planAvailableState = new PlanAvailableState(ros2Helper, ros2Node, simpleRobotName);
      State waitingtoLandState = new WaitingToLandState(ros2Helper, simpleRobotName);
      continuousHikingMode = new YoEnum<>("ContinuousHikingMode", registry, ContinuousHikingState.class, true);

      factory.addState(ContinuousHikingState.NOT_STARTED, notStartedState);
      factory.addState(ContinuousHikingState.PLAN_AVAILABLE, planAvailableState);
      factory.addState(ContinuousHikingState.WAITING_TO_LAND, waitingtoLandState);
      factory.addState(ContinuousHikingState.READY_TO_PLAN, readyToPlanState);

      factory.addRequestedTransition(ContinuousHikingState.NOT_STARTED, continuousHikingMode);
      factory.addRequestedTransition(ContinuousHikingState.PLAN_AVAILABLE, continuousHikingMode);
      factory.addRequestedTransition(ContinuousHikingState.WAITING_TO_LAND, continuousHikingMode);
      factory.addRequestedTransition(ContinuousHikingState.READY_TO_PLAN, continuousHikingMode);

      stateMachine = factory.build(ContinuousHikingState.NOT_STARTED);

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousHikingState
    */
   private void tickStateMachine()
   {
      continuousPlanner.syncParametersCallback();
      if (!parameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         if (stateMachine.getCurrentStateKey() != ContinuousHikingState.NOT_STARTED)
            stateMachine.resetToInitialState();
      }

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

   private class NotStartedState implements State
   {
      private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

      public NotStartedState(ROS2Node ros2Node, String simpleRobotName)
      {
         pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
      }

      @Override
      public void onEntry()
      {
         LogTools.warn(continuousHikingMode.getEnumValue());
      }

      @Override
      public void doAction(double timeInState)
      {
         if (!parameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
         {
            continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);

            stepCounter = 0;
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
               continuousHikingMode.set(ContinuousHikingState.PLAN_AVAILABLE);
            }
            else
            {
               continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);
               continuousPlanner.setInitialized(false);

               LogTools.error(message = String.format("State: [%s]: Initialization failed... will retry initializing next tick",
                                                      continuousHikingMode.getEnumValue()));
               statistics.appendString(message);
            }
         }
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private class ReadyToPlanState implements State
   {
      private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();
      private List<QueuedFootstepStatusMessage> controllerQueue;

      public ReadyToPlanState(ROS2Helper ros2Helper, String simpleRobotName)
      {
         ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
         ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      }

      private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
      {
         if (!parameters.getEnableContinuousWalking())
            return;

         controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
         if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         {
            LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                                  continuousHikingMode.getEnumValue()));
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
         LogTools.warn(continuousHikingMode.getEnumValue());
      }

      @Override
      public void doAction(double timeInState)
      {
         statistics.setLastAndTotalWaitingTimes();

         if (parameters.getStepPublisherEnabled())
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
            continuousHikingMode.set(ContinuousHikingState.PLAN_AVAILABLE);
         }
         else
         {
            // TODO: Add replanning. Replanned plan valid only until the foot lands.
            continuousHikingMode.set(ContinuousHikingState.WAITING_TO_LAND);
            LogTools.error(message = String.format("State: [%s]: Planning failed... will try again when current step is completed",
                                                   continuousHikingMode.getEnumValue()));
            statistics.appendString(message);
         }
      }

      @Override
      public void onExit(double timeInState)
      {
         if (!parameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
            continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);
      }
   }

   private class PlanAvailableState implements State
   {
      private List<QueuedFootstepStatusMessage> controllerQueue;
      private final ROS2Topic controllerFootstepDataTopic;
      private final ROS2PublisherMap publisherMap;

      public PlanAvailableState(ROS2Helper ros2Helper, ROS2Node ros2Node, String simpleRobotName)
      {
         ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);

         controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
         publisherMap = new ROS2PublisherMap(ros2Node);
         publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);
      }

      private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
      {
         if (!parameters.getEnableContinuousWalking())
            return;

         controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
         if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         {
            LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                                  continuousHikingMode.getEnumValue()));
         }
         controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
      }

      @Override
      public void onEntry()
      {
         LogTools.warn(continuousHikingMode.getEnumValue());
      }

      @Override
      public void doAction(double timeInState)
      {
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(parameters, controllerQueue);

         debugger.publishPlannedFootsteps(footstepDataList);
         if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get()
                                                                                                                                   .getUseMonteCarloPlanAsReference())
         {
            debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
            debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
         }

         if (parameters.getStepPublisherEnabled())
         {
            LogTools.info(message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller",
                                                  continuousHikingMode.getEnumValue()));

            FramePose3DReadOnly stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), continuousPlanner.getImminentFootstepPose());

            FramePose3DReadOnly candidateStepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                    footstepDataList.getFootstepDataList().get(0).getLocation(),
                                                                    footstepDataList.getFootstepDataList().get(0).getOrientation());

            BipedalFootstepPlannerNodeRejectionReason reason;
            FramePose3D startOfSwingPose = new FramePose3D();

            RobotSide stepSide = continuousPlanner.getImminentFootstepSide();
            assert stepSide != null;
            startOfSwingPose.setFromReferenceFrame(referenceFrames.getSoleFrame(stepSide.getOppositeSide()));
            reason = stepChecker.checkValidity(stepSide.getOppositeSide(), candidateStepPose, stanceFootPose, startOfSwingPose);

            if (reason == null && stepCounter < MAX_STEPS_PER_SESSION)
            {
               publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

               continuousHikingMode.set(ContinuousHikingState.WAITING_TO_LAND);
               continuousPlanner.setPlanAvailable(false);
               continuousPlanner.transitionCallback();
               statistics.setStartWaitingTime();
            }
            else if (stepCounter >= MAX_STEPS_PER_SESSION)
            {
               LogTools.warn("[REASON]: Max Number of Steps Reached");
            }
            else if (reason != null)
            {
               parameters.setEnableContinuousWalking(false);
               LogTools.error("Planning failed:" + reason.name());
               LogTools.info("Start of Swing:" + startOfSwingPose);
               LogTools.info("Stance:" + stanceFootPose);
               LogTools.info("Candidate:" + candidateStepPose);

               continuousPlanner.setInitialized(false);
               continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);
            }
         }
         else
         {
            continuousHikingMode.set(ContinuousHikingState.READY_TO_PLAN);
         }
      }

      @Override
      public void onExit(double timeInState)
      {
         if (!parameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
            continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);
      }
   }

   private class WaitingToLandState implements State
   {
      private final AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>();

      public WaitingToLandState(ROS2Helper ros2Helper, String simpleRobotName)
      {
         ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      }

      /*
       * Callback to receive a message each time there is a change in the FootstepStatusMessage
       */
      private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
      {
         if (!parameters.getEnableContinuousWalking())
            return;

         if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            stepCounter++;
            continuousHikingMode.set(ContinuousHikingState.READY_TO_PLAN);
            statistics.endStepTime();
            statistics.startStepTime();
         }
         else if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            // TODO: Use the transfer time (starting now) to start planning (if WAITING_TO_LAND then plan again)
            statistics.setLastFootstepQueueLength(controllerQueueSize);
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
         LogTools.warn(continuousHikingMode.getEnumValue());
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
         if (!parameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
            continuousHikingMode.set(ContinuousHikingState.NOT_STARTED);
      }
   }
}