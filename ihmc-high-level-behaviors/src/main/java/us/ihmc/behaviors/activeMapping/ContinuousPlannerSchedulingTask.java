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
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
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
    * This limits the number of steps in a session, will cause the state machine to stop
    */
   private final static int MAX_STEPS_PER_SESSION = 50;

   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   private ContinuousHikingState state = ContinuousHikingState.NOT_STARTED;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
   private final ROS2Topic controllerFootstepDataTopic;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2PublisherMap publisherMap;
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
   private List<QueuedFootstepStatusMessage> controllerQueue;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousHikingParameters parameters,
                                          ContinuousPlanner.PlanningMode mode)
   {
      this.referenceFrames = referenceFrames;
      this.parameters = parameters;
      String simpleRobotName = robotModel.getSimpleRobotName();

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);
      pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
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
         state = ContinuousHikingState.NOT_STARTED;

         stepCounter = 0;
         sendPauseWalkingMessage();
         setImminentStanceToCurrent();

         continuousPlanner.setInitialized(false);
         continuousPlanner.requestMonteCarloPlannerReset();

         return;
      }

      if (!continuousPlanner.isInitialized())
      {
         statistics.appendString("Restarting State Machine");
         initializeContinuousPlanner();
      }
      else
      {
         handleStateMachine();
      }
   }

   private void sendPauseWalkingMessage()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();

      if (continuousPlanner.isInitialized())
      {
         message.setPause(true);
         message.setClearRemainingFootstepQueue(true);
         pauseWalkingPublisher.publish(message);
      }
   }

   public void initializeContinuousPlanner()
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
         state = ContinuousHikingState.PLAN_AVAILABLE;
      }
      else
      {
         state = ContinuousHikingState.NOT_STARTED;
         continuousPlanner.setInitialized(false);

         LogTools.error(message = String.format("State: [%s]: Initialization failed... will retry initializing next tick", state));
         statistics.appendString(message);
      }
   }

   public void handleStateMachine()
   {
      /*
       * Ready to plan means that the current step is completed and the planner is ready to plan the next step
       */
      if (state == ContinuousHikingState.READY_TO_PLAN)
      {
         LogTools.info("State: " + state);
         statistics.setLastAndTotalWaitingTimes();

         if (parameters.getStepPublisherEnabled())
         {
            continuousPlanner.getImminentStanceFromLatestStatus(footstepStatusMessage, controllerQueue);
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
            state = ContinuousHikingState.PLAN_AVAILABLE;
         }
         else
         {
            // TODO: Add replanning. Replanned plan valid only until the foot lands.
            state = ContinuousHikingState.WAITING_TO_LAND;
            LogTools.error(message = String.format("State: [%s]: Planning failed... will try again when current step is completed", state));
            statistics.appendString(message);
         }
      }

      /*
       * Plan available means that the planner has a plan ready to send to the controller
       */
      if (state == ContinuousHikingState.PLAN_AVAILABLE)
      {
         LogTools.info("State: " + state);
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
            LogTools.info(message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller", state));

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

               state = ContinuousHikingState.WAITING_TO_LAND;
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
               state = ContinuousHikingState.NOT_STARTED;
            }
         }
         else
         {
            state = ContinuousHikingState.READY_TO_PLAN;
         }
      }
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
         state = ContinuousHikingState.READY_TO_PLAN;
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

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (!parameters.getEnableContinuousWalking())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                               state));
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void setImminentStanceToCurrent()
   {
      RobotSide closerSide = continuousPlanner.getCloserSideToGoal();
      FramePose3D closerToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide));
      FramePose3D fartherToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide.getOppositeSide()));
      closerToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      fartherToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      if (continuousPlanner.updateImminentStance(fartherToGoalFootPose, closerToGoalFootPose, closerSide))
      {
         debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      }
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