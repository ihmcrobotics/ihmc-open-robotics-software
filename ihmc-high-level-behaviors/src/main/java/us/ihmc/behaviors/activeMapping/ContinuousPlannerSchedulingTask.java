package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.communication.video.ContinuousPlanningAPI;
import us.ihmc.euclid.geometry.Pose3D;
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
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   private ContinuousWalkingState state = ContinuousWalkingState.NOT_STARTED;
   private enum ContinuousWalkingState
   {
      NOT_STARTED, READY_TO_PLAN, PLAN_AVAILABLE, WAITING_TO_LAND
   }

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final ROS2Topic controllerFootstepDataTopic;
   private final IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   private final IHMCROS2Publisher<PoseListMessage> startAndGoalPublisherForUI;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2PublisherMap publisherMap;

   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat contactMapImage;

   private ContinuousPlannerStatistics continuousPlannerStatistics;
   private final ContinuousWalkingParameters continuousPlanningParameters;

   private final HumanoidReferenceFrames referenceFrames;

   private final ContinuousPlanner continuousPlanner;

   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousWalkingParameters continuousPlanningParameters)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanningParameters = continuousPlanningParameters;

      controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      publisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousPlanningAPI.PLANNED_FOOTSTEPS);
      startAndGoalPublisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousPlanningAPI.START_AND_GOAL_FOOTSTEPS);
      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, inputTopic);

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepQueueStatusReceived);

      continuousPlanner = new ContinuousPlanner(robotModel,
                                                referenceFrames,
                                                ContinuousPlanner.PlanningMode.WALK_TO_GOAL);

      continuousPlannerStatistics = new ContinuousPlannerStatistics();
      continuousPlanner.setContinuousPlannerStatistics(continuousPlannerStatistics);

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void tickStateMachine()
   {
      if (!continuousPlanningParameters.getContinuousWalkingEnabled())
      {
         stopContinuousWalkingGracefully();

         state = ContinuousWalkingState.NOT_STARTED;
         return;
      }

      if (!continuousPlanner.isInitialized())
      {
         initializeContinuousPlanner();
      }
      else
      {
         planAndSendFootsteps();
      }
   }

   private void stopContinuousWalkingGracefully()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();

      if (continuousPlanner.isInitialized())
      {
         message.setPause(true);
         pauseWalkingPublisher.publish(message);
      }

      continuousPlanner.setInitialized(false);
   }

   public void initializeContinuousPlanner()
   {
      continuousPlanner.initialize();
      continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
      continuousPlanner.planToGoalWithHeightMap(latestHeightMapData, heightMapImage, contactMapImage, false);

      if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
      {
         state = ContinuousWalkingState.PLAN_AVAILABLE;
      }
      else
      {
         //continuousPlanningParameters.setContinuousWalkingEnabled(false);
         continuousPlanningParameters.setStepPublisherEnabled(false);
         LogTools.error("Planning failed, restart active mapping and try again");
      }
   }

   public void planAndSendFootsteps()
   {
      if (state == ContinuousWalkingState.READY_TO_PLAN)
      {
         LogTools.info("State: " + state);
         continuousPlannerStatistics.setLastAndTotalWaitingTimes();
         continuousPlanner.getImminentStanceFromLatestStatus(footstepStatusMessage, controllerQueue);
         publishForVisualization();
         continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
         continuousPlanner.planToGoalWithHeightMap(latestHeightMapData, heightMapImage, contactMapImage, true);

         if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
         {
            state = ContinuousWalkingState.PLAN_AVAILABLE;
         }
         else
         {
            continuousPlanningParameters.setContinuousWalkingEnabled(false);
            LogTools.error("Planning failed, restart active mapping and try again");
         }
      }

      if (state == ContinuousWalkingState.PLAN_AVAILABLE)
      {
         LogTools.info("State: " + state);
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousPlanningParameters, controllerQueue);
         LogTools.info("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");

         if (continuousPlanningParameters.getStepPublisherEnabled())
         {
            publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

            state = ContinuousWalkingState.WAITING_TO_LAND;
            continuousPlanner.setPlanAvailable(false);
            continuousPlanner.transitionCallback();
            continuousPlannerStatistics.setStartWaitingTime();
         }
      }
   }

   // This receives a message each time there is a change in the FootstepStatusMessage
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (!continuousPlanningParameters.getContinuousWalkingEnabled())
         return;

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         state = ContinuousWalkingState.READY_TO_PLAN;
         continuousPlannerStatistics.endStepTime();
         continuousPlannerStatistics.startStepTime();
      }
      else if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         continuousPlannerStatistics.setLastFootstepQueueLength(controllerQueueSize);
         continuousPlannerStatistics.incrementTotalStepsCompleted();

         double distance = referenceFrames.getSoleFrame(RobotSide.LEFT)
                                          .getTransformToDesiredFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT))
                                          .getTranslation()
                                          .norm();
         continuousPlannerStatistics.setLastLengthCompleted((float) distance);

         continuousPlannerStatistics.logToFile(true, true);
      }

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (!continuousPlanningParameters.getContinuousWalkingEnabled())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         LogTools.warn("Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   public void publishForVisualization()
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(continuousPlanner.getStartingStancePose().get(RobotSide.LEFT)));
      poses.add(new Pose3D(continuousPlanner.getStartingStancePose().get(RobotSide.RIGHT)));
      poses.add(new Pose3D(continuousPlanner.getGoalStancePose().get(RobotSide.LEFT)));
      poses.add(new Pose3D(continuousPlanner.getGoalStancePose().get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      startAndGoalPublisherForUI.publish(poseListMessage);
   }

   public void setLatestHeightMapData(HeightMapData heightMapData)
   {
      this.latestHeightMapData = new HeightMapData(heightMapData);
   }

   public void setHeightMapImage(Mat heightMapImage)
   {
      this.heightMapImage = heightMapImage.clone();
   }

   public void setContactMapImage(Mat contactMapImage)
   {
      this.contactMapImage = contactMapImage.clone();
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
