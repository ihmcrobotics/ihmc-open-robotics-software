package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
 * This class is responsible for scheduling the continuous planner state machine. It is responsible for handling the state transitions and the logic of the state machine.
 */
public class ContinuousPlannerSchedulingTask
{
   /**
    * This limits the number of steps in a session, will cause the state machine to stop
    * */
   private final static int MAX_STEPS_PER_SESSION = 50;

   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   private ContinuousWalkingState state = ContinuousWalkingState.NOT_STARTED;

   private enum ContinuousWalkingState
   {
      NOT_STARTED, READY_TO_PLAN, NEED_TO_REPLAN, PLAN_AVAILABLE, WAITING_TO_LAND, PAUSED
   }

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
   private final ROS2Topic controllerFootstepDataTopic;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2PublisherMap publisherMap;
   private HeightMapData latestHeightMapData;
   private TerrainMapData terrainMap;

   private final ContinuousPlannerStatistics statistics;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final FootstepPoseHeuristicChecker stepChecker;

   private final ContinuousPlanner continuousPlanner;
   private final HumanoidReferenceFrames referenceFrames;

   private String message = "";
   private int stepCounter = 0;
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Helper ros2Helper,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousHikingParameters continuousHikingParameters,
                                          ContinuousPlanner.PlanningMode mode)
   {
      String simpleRobotName = robotModel.getSimpleRobotName();
      this.referenceFrames = referenceFrames;
      this.continuousHikingParameters = continuousHikingParameters;

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND, commandMessage::set);

      SideDependentList<ConvexPolygon2D> footPolygons = FootstepPlanningModuleLauncher.createFootPolygons(robotModel);
      statistics = new ContinuousPlannerStatistics();
      continuousPlanner = new ContinuousPlanner(robotModel, ros2Node, referenceFrames, footPolygons, mode, this.continuousHikingParameters);
      continuousPlanner.setContinuousPlannerStatistics(statistics);

      // FIXME this needs to get a copy of the height map or terrain map for the step checker to actually work as intended.
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, continuousPlanner.getFootstepPlannerParameters(), environmentHandler);
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      stepChecker = new FootstepPoseHeuristicChecker(continuousPlanner.getFootstepPlannerParameters(), snapper, registry);

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void tickStateMachine()
   {

      continuousPlanner.syncParametersCallback();

      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         state = ContinuousWalkingState.NOT_STARTED;

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
      if (commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get().getUseMonteCarloPlanAsReference() || commandMessage.get().getUseHybridPlanner())
      {
         continuousPlanner.getDebugger().publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
         continuousPlanner.getDebugger().publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
      }

      if (continuousPlanner.isPlanAvailable())
      {
         state = ContinuousWalkingState.PLAN_AVAILABLE;
      }
      else
      {
         state = ContinuousWalkingState.NOT_STARTED;
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
      if (state == ContinuousWalkingState.READY_TO_PLAN)
      {
         LogTools.info("State: " + state);
         statistics.setLastAndTotalWaitingTimes();

         if (continuousHikingParameters.getStepPublisherEnabled())
         {
            continuousPlanner.getImminentStanceFromLatestStatus(footstepStatusMessage, controllerQueue);
         }

         continuousPlanner.getDebugger().publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
         continuousPlanner.setGoalWaypointPoses();
         continuousPlanner.planToGoal(commandMessage.get());

         if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get().getUseMonteCarloPlanAsReference())
         {
            continuousPlanner.getDebugger().publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
            continuousPlanner.getDebugger().publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
         }

         if (continuousPlanner.isPlanAvailable())
         {
            state = ContinuousWalkingState.PLAN_AVAILABLE;
         }
         else
         {
            state = ContinuousWalkingState.WAITING_TO_LAND;
            LogTools.error(message = String.format("State: [%s]: Planning failed... will try again when current step is completed", state));
            statistics.appendString(message);
         }
      }

      /*
       * Plan available means that the planner has a plan ready to send to the controller
       */
      if (state == ContinuousWalkingState.PLAN_AVAILABLE)
      {
         LogTools.info("State: " + state);
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters, controllerQueue);

         continuousPlanner.getDebugger().publishPlannedFootsteps(footstepDataList);
         if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get().getUseMonteCarloPlanAsReference())
         {
            continuousPlanner.getDebugger().publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
            continuousPlanner.getDebugger().publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
         }

         if (continuousHikingParameters.getStepPublisherEnabled())
         {
            LogTools.info(message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller", state));


            FramePose3DReadOnly stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                 continuousPlanner.getImminentFootstepPose());

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

               state = ContinuousWalkingState.WAITING_TO_LAND;
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
               continuousHikingParameters.setEnableContinuousWalking(false);
               LogTools.error("Planning failed:" + reason.name());
               LogTools.info("Start of Swing:" + startOfSwingPose);
               LogTools.info("Stance:" + stanceFootPose);
               LogTools.info("Candidate:" + candidateStepPose);

               continuousPlanner.setInitialized(false);
               state = ContinuousWalkingState.NOT_STARTED;
            }
         }
         else
         {
            state = ContinuousWalkingState.READY_TO_PLAN;
         }
      }
   }

   /*
    * Callback to receive a message each time there is a change in the FootstepStatusMessage
    */
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         stepCounter++;
         state = ContinuousWalkingState.READY_TO_PLAN;
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
      if (!continuousHikingParameters.getEnableContinuousWalking())
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
         continuousPlanner.getDebugger().publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      }
   }

   public void setLatestHeightMapData(HeightMapData heightMapData)
   {
      this.latestHeightMapData = new HeightMapData(heightMapData);
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
}
