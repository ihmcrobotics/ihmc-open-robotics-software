package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.communication.video.ContinuousPlanningAPI;
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

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlannerSchedulingTask
{
   private final static long CONTINUOUS_PLANNING_DELAY_BEFORE_NEXT_LOOP_MS = 32;

   private enum ContinuousWalkingState
   {
      NOT_STARTED, FOOTSTEP_STARTED, PLANNING_FAILED, FOOTSTEP_COMPLETED
   }

   private ContinuousWalkingState continuousPlannerState = ContinuousWalkingState.NOT_STARTED;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());

   private ROS2PublisherMap publisherMap;
   private HeightMapData latestHeightMapData;

   private final ROS2Topic controllerFootstepDataTopic;
   private final ContinuousPlanningParameters continuousPlanningParameters;
   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousPlanner continuousPlanner;

   private int controllerFootstepQueueSize = 0;

   IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   IHMCROS2Publisher<RigidBodyTransformMessage> publisherForGoalPoses;
   IHMCROS2Publisher<RigidBodyTransformMessage> publisherForStartPoses;

   public ContinuousPlannerSchedulingTask(DRCRobotModel robotModel,
                                          ROS2Node ros2Node,
                                          HumanoidReferenceFrames referenceFrames,
                                          ContinuousPlanningParameters continuousPlanningParameters)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanningParameters = continuousPlanningParameters;
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, ContinuousPlanner.PlanningMode.WALK_TO_GOAL);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      publisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousPlanningAPI.PLANNED_FOOTSTEPS);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::footstepQueueStatusReceived);

      executorService.scheduleWithFixedDelay(this::updateContinuousPlanner, 1500, CONTINUOUS_PLANNING_DELAY_BEFORE_NEXT_LOOP_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Set up the continuous planner, keep the orientation of the MidFeetZUpFrame as that is the direction the robot will try and walk
    */
   public void initializeContinuousPlanner()
   {
      continuousPlanner.initialize();
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void updateContinuousPlanner()
   {
      // Reset the continuous planner, so when its enabled again it starts normally
      if (!continuousPlanningParameters.getActiveMapping())
      {
         continuousPlanner.setInitialized(false);
         return;
      }

      // Initialize the continuous planner so that the state machine starts in the correct configuration
      if (!continuousPlanner.isInitialized())
      {
         initializeContinuousPlanner();
         continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
         continuousPlanner.planToGoalWithHeightMap(latestHeightMapData);

         if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;
         else
            continuousPlannerState = ContinuousWalkingState.PLANNING_FAILED;
      }
      else if (!continuousPlanningParameters.getPauseContinuousWalking())
      {
         // The state machine will always run this method if the continuous planner is initialized and not paused
         planAndSendFootsteps();
      }
   }

   public void planAndSendFootsteps()
   {
      // A foot is in swing, plan another step
      if ((footstepStatusMessage.get().getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED
           && continuousPlannerState == ContinuousWalkingState.FOOTSTEP_COMPLETED) || continuousPlannerState == ContinuousWalkingState.PLANNING_FAILED)
      {
         getImminentStanceFromLatestStatus();
         continuousPlanner.setGoalWaypointPoses(continuousPlanningParameters);
         continuousPlanner.planToGoalWithHeightMap(latestHeightMapData);

         if (continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION || continuousPlanner.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;
         else
            continuousPlannerState = ContinuousWalkingState.PLANNING_FAILED;
      }
      // If we got a plan from the last time the planner ran, and we are running low on footsteps in the queue, send this plan
      else if (continuousPlanner.isPlanAvailable() && continuousPlannerState == ContinuousWalkingState.FOOTSTEP_STARTED)
      {
         if (controllerFootstepQueueSize == 0)
         {
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_COMPLETED;
         }
         else
         {
            continuousPlannerState = ContinuousWalkingState.NOT_STARTED;
         }

         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousPlanningParameters.getNumberOfStepsToSend(),
                                                                                                        (float) continuousPlanningParameters.getSwingTime(),
                                                                                                        (float) continuousPlanningParameters.getTransferTime());
         LogTools.info("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
         publisherMap.publish(controllerFootstepDataTopic, footstepDataList);
         publisherForUI.publish(footstepDataList);
         continuousPlanner.setPreviouslySentPlanForReference();

         continuousPlannerState = ContinuousWalkingState.NOT_STARTED;
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
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         continuousPlannerState = ContinuousWalkingState.FOOTSTEP_COMPLETED;

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (controllerFootstepQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         LogTools.warn("Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
      controllerFootstepQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
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
