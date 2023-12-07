package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
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
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;

   private ContinuousWalkingState state = ContinuousWalkingState.NOT_STARTED;

   private enum ContinuousWalkingState
   {
      NOT_STARTED, READY_TO_PLAN, NEED_TO_REPLAN, PLAN_AVAILABLE, WAITING_TO_LAND
   }

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousWalkingCommandMessage());
   private final ROS2Topic controllerFootstepDataTopic;
   private final IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   private final IHMCROS2Publisher<PoseListMessage> startAndGoalPublisherForUI;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2PublisherMap publisherMap;

   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat contactMapImage;

   private ContinuousPlannerStatistics continuousPlannerStatistics;
   private final ContinuousWalkingParameters continuousWalkingParameters;

   private final HumanoidReferenceFrames referenceFrames;

   private final ContinuousPlanner continuousPlanner;

   private String message = "";
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousWalkingParameters continuousWalkingParameters,
                                          ContinuousPlanner.PlanningMode mode)
   {
      this.referenceFrames = referenceFrames;
      this.continuousWalkingParameters = continuousWalkingParameters;

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
      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.CONTINUOUS_WALKING_COMMAND, commandMessage::set);

      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, mode);

      continuousPlannerStatistics = new ContinuousPlannerStatistics();
      continuousPlanner.setContinuousPlannerStatistics(continuousPlannerStatistics);

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   FramePose3D rightRobotFoot;
   FramePose3D leftRobotFoot;

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void tickStateMachine()
   {
      // Sets the planner timeout to be a percentage of the total step duration
      //      double stepDuration = continuousWalkingParameters.getSwingTime() + continuousWalkingParameters.getTransferTime();
      //      continuousWalkingParameters.setPlanningReferenceTimeout(stepDuration * continuousWalkingParameters.getPlannerTimeoutFraction());

      if (!continuousWalkingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         stopContinuousWalkingGracefully();

         state = ContinuousWalkingState.NOT_STARTED;

         rightRobotFoot = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         leftRobotFoot = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                          referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());

         if (continuousPlanner.updateImminentStance(rightRobotFoot, leftRobotFoot, RobotSide.RIGHT))
            publishStartAndGoalForVisualization();
         return;
      }

      if (!continuousPlanner.isInitialized())
      {
         LogTools.info("Restarting State Machine");
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
      continuousPlanner.setGoalWaypointPoses(continuousWalkingParameters);
      continuousPlanner.planToGoalWithHeightMap(latestHeightMapData, heightMapImage, contactMapImage, false);

      if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION
          || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
      {
         state = ContinuousWalkingState.PLAN_AVAILABLE;
      }
      else
      {
         state = ContinuousWalkingState.NOT_STARTED;
         continuousPlanner.setInitialized(false);

         LogTools.error(message = String.format("State: [%s]: Initialization failed... will retry initializing next tick", state));
         continuousPlannerStatistics.appendString(message);
      }
   }

   public void planAndSendFootsteps()
   {
      /*
       * Ready to plan means that the current step is completed and the planner is ready to plan the next step
       */
      if (state == ContinuousWalkingState.READY_TO_PLAN)
      {
         LogTools.info("State: " + state);
         continuousPlannerStatistics.setLastAndTotalWaitingTimes();

         if (continuousWalkingParameters.getStepPublisherEnabled())
            continuousPlanner.getImminentStanceFromLatestStatus(footstepStatusMessage, controllerQueue);

         publishStartAndGoalForVisualization();
         continuousPlanner.setGoalWaypointPoses(continuousWalkingParameters);
         continuousPlanner.planToGoalWithHeightMap(latestHeightMapData,
                                                   heightMapImage,
                                                   contactMapImage,
                                                   true);

         if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION
             || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
         {
            state = ContinuousWalkingState.PLAN_AVAILABLE;
         }
         else
         {
            state = ContinuousWalkingState.WAITING_TO_LAND;
            LogTools.error(message = String.format("State: [%s]: Planning failed... will try again when current step is completed", state));
            continuousPlannerStatistics.appendString(message);
         }
      }

      /*
       * Plan available means that the planner has a plan ready to send to the controller
       */
      if (state == ContinuousWalkingState.PLAN_AVAILABLE)
      {
         LogTools.info("State: " + state);
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousWalkingParameters, controllerQueue);

         publisherForUI.publish(footstepDataList);
         if (continuousWalkingParameters.getStepPublisherEnabled())
         {
            LogTools.info(message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller", state));
            publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

            state = ContinuousWalkingState.WAITING_TO_LAND;
            continuousPlanner.setPlanAvailable(false);
            continuousPlanner.transitionCallback();
            continuousPlannerStatistics.setStartWaitingTime();
         }
         else
         {
            state = ContinuousWalkingState.NOT_STARTED;
            continuousPlanner.setInitialized(false);
         }
      }
   }

   // This receives a message each time there is a change in the FootstepStatusMessage
   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (!continuousWalkingParameters.getEnableContinuousWalking())
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
      if (!continuousWalkingParameters.getEnableContinuousWalking())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                               state));
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   public void publishStartAndGoalForVisualization()
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
