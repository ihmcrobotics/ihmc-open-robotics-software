package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.communication.video.ContinuousPlanningAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlannerSchedulingTask
{
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 10;

   private enum ContinuousWalkingState
   {
      NOT_STARTED, READY_TO_PLAN, PLANNING_FAILED, PLAN_AVAILABLE, WAITING_TO_LAND
   }

   private ContinuousWalkingState state = ContinuousWalkingState.NOT_STARTED;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final ContinuousPlannerStatistics continuousPlannerStatistics = new ContinuousPlannerStatistics();

   private ROS2PublisherMap publisherMap;
   private HeightMapData latestHeightMapData;

   private final ROS2Topic controllerFootstepDataTopic;
   private final ContinuousPlanningParameters continuousPlanningParameters;
   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousPlanner continuousPlanner;

   private int controllerFootstepQueueSize = 0;

   IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   IHMCROS2Publisher<PoseListMessage> startAndGoalPublisherForUI;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousPlanningParameters continuousPlanningParameters)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanningParameters = continuousPlanningParameters;
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, continuousPlannerStatistics, ContinuousPlanner.PlanningMode.WALK_TO_GOAL);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      publisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousPlanningAPI.PLANNED_FOOTSTEPS);
      startAndGoalPublisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousPlanningAPI.START_AND_GOAL_FOOTSTEPS);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepQueueStatusReceived);

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void tickStateMachine()
   {
      if (!continuousPlanningParameters.getActiveMapping())
      {
         continuousPlanner.setInitialized(false);
         state = ContinuousWalkingState.NOT_STARTED;
         return;
      }

      if (!continuousPlanner.isInitialized())
      {
         initializeContinuousPlanner();
      }
      else if (!continuousPlanningParameters.getPauseContinuousWalking())
      {
         planAndSendFootsteps();
      }

      LogTools.info(continuousPlannerStatistics.toString());
   }

   public void initializeContinuousPlanner()
   {
      continuousPlanner.initialize();
      continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
      continuousPlanner.planToGoalWithHeightMap(latestHeightMapData);

      if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
         state = ContinuousWalkingState.PLAN_AVAILABLE;
      else
         state = ContinuousWalkingState.PLANNING_FAILED;
   }

   public void planAndSendFootsteps()
   {
      if (state == ContinuousWalkingState.READY_TO_PLAN || state == ContinuousWalkingState.PLANNING_FAILED)
      {
         LogTools.info("State: " + state);
         getImminentStanceFromLatestStatus();
         continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
         continuousPlanner.planToGoalWithHeightMap(latestHeightMapData);

         if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
            state = ContinuousWalkingState.PLAN_AVAILABLE;
         else
            state = ContinuousWalkingState.PLANNING_FAILED;
      }

      if (state == ContinuousWalkingState.PLAN_AVAILABLE)
      {
         LogTools.info("State: " + state);
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousPlanningParameters.getNumberOfStepsToSend(),
                                                                                                        (float) continuousPlanningParameters.getSwingTime(),
                                                                                                        (float) continuousPlanningParameters.getTransferTime());
         LogTools.info("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
         publisherMap.publish(controllerFootstepDataTopic, footstepDataList);
         publishForVisualization(footstepDataList);
         continuousPlanner.setPreviouslySentPlanForReference();

         state = ContinuousWalkingState.WAITING_TO_LAND;
         continuousPlanner.setPlanAvailable(false);
      }
   }

   /**
    * This method gets called when we need a new footstep plan, this gets the latest information in the controller footstep queue
    */
   public void getImminentStanceFromLatestStatus()
   {
      RobotSide imminentFootstepSide = RobotSide.fromByte(footstepStatusMessage.get().getRobotSide());
      assert imminentFootstepSide != null;
      FramePose3D realRobotStancePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                        referenceFrames.getSoleFrame(imminentFootstepSide.getOppositeSide()).getTransformToWorldFrame());
      FramePose3D imminentFootstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                         footstepStatusMessage.get().getDesiredFootPositionInWorld(),
                                                         footstepStatusMessage.get().getDesiredFootOrientationInWorld());
      continuousPlanner.updateImminentStance(realRobotStancePose, imminentFootstepPose, imminentFootstepSide);
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         state = ContinuousWalkingState.READY_TO_PLAN;
         continuousPlannerStatistics.startStepTime();
      }

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         continuousPlannerStatistics.endStepTime();
         continuousPlannerStatistics.incrementTotalStepsCompleted();
      }

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (controllerFootstepQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         LogTools.warn("Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
      controllerFootstepQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   public void publishForVisualization(FootstepDataListMessage footstepDataList)
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(continuousPlanner.getStartingStancePose().get(RobotSide.LEFT)));
      poses.add(new Pose3D(continuousPlanner.getStartingStancePose().get(RobotSide.RIGHT)));
      poses.add(new Pose3D(continuousPlanner.getGoalStancePose().get(RobotSide.LEFT)));
      poses.add(new Pose3D(continuousPlanner.getGoalStancePose().get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      startAndGoalPublisherForUI.publish(poseListMessage);
      publisherForUI.publish(footstepDataList);
   }

   public void setLatestHeightMapData(HeightMapData heightMapData)
   {
      this.latestHeightMapData = new HeightMapData(heightMapData);
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
