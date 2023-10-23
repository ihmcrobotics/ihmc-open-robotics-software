package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.ByteBuffer;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlanningRemoteTask
{
   private final static long CONTINUOUS_PLANNING_DELAY_BEFORE_NEXT_LOOP_MS = 50;
   private final static float SWING_DURATION = 0.8f;
   private final static float TRANSFER_DURATION = 0.4f;
   private final static int MAXIMUM_FOOTSTEPS_TO_SEND = 1;
   private final static int MAXIMUM_FOOTSTEPS_HELD_IN_CONTROLLER_QUEUE = 2;

   private enum ContinuousWalkingState
   {
      NOT_STARTED,
      FOOTSTEP_STARTED,
      PLANNING_FAILED
   }

   private ContinuousWalkingState continuousPlannerState = ContinuousWalkingState.NOT_STARTED;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                   getClass(),
                                                                   ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());

   private final ROS2PublisherMap publisherMap;
   private final ROS2Topic controllerFootstepDataTopic;

   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private final ContinuousPlanningParameters continuousPlanningParameters;
   private final HumanoidReferenceFrames referenceFrames;

   private final ContinuousPlanner continuousPlanner;
   private FootstepPlannerOutput plannerOutput;

   private FixedReferenceFrame originalReferenceFrameToBaseGoalPoseDirectionFrom;
   private final SideDependentList<FramePose3D> originalPoseToBaseGoalPoseFrom = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private SideDependentList<FramePose3D> startPoseForFootstepPlanner = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalPoseForFootstepPlanner = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final FramePose3D firstImminentFootstep = new FramePose3D();
   private final FramePose3D secondImminentFootstep = new FramePose3D();
   private RobotSide secondImminentFootstepSide = RobotSide.LEFT;

   private int multiplierForGoalPoseDistance;
   private int queuedFootstepSize = 0;
   private List<QueuedFootstepStatusMessage> queuedFootstepList;
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private long originalFootstepDataListId = -1;

   public ContinuousPlanningRemoteTask(DRCRobotModel robotModel,
                                       ROS2Node ros2Node,
                                       HumanoidReferenceFrames referenceFrames,
                                       ContinuousPlanningParameters continuousPlanningParameters)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanningParameters = continuousPlanningParameters;
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, ContinuousPlanner.PlanningMode.MAX_COVERAGE);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      robotModel.getLookAndStepParameters();

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepQueueStatusReceived);

      executorService.scheduleWithFixedDelay(this::updateContinuousPlanner, 1500,
                                             CONTINUOUS_PLANNING_DELAY_BEFORE_NEXT_LOOP_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Set up the continuous planner, keep the orientation of the MidFeetZUpFrame as that is the direction the robot will try and walk
    */
   public void initializeContinuousPlanner()
   {
      originalReferenceFrameToBaseGoalPoseDirectionFrom = new FixedReferenceFrame("Original Reference Frame", ReferenceFrame.getWorldFrame(), referenceFrames.getMidFeetZUpFrame().getTransformToParent());

      for (RobotSide side : RobotSide.values)
      {
         startPoseForFootstepPlanner.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side)) ;
         originalPoseToBaseGoalPoseFrom.get(side).getPosition().setX(startPoseForFootstepPlanner.get(side).getPosition().getX());
         originalPoseToBaseGoalPoseFrom.get(side).getPosition().setY(startPoseForFootstepPlanner.get(side).getPosition().getY());
         originalPoseToBaseGoalPoseFrom.get(side).getPosition().setZ(startPoseForFootstepPlanner.get(side).getPosition().getZ());
         originalPoseToBaseGoalPoseFrom.get(side).getOrientation().setToYawOrientation(referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame().getRotation().getYaw());
      }
      continuousPlanner.setInitialized(true);
   }

   /**
    * Runs the continuous planner state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The state is stored in the ContinuousWalkingState
    */
   private void updateContinuousPlanner()
   {
      // Reset the continuous planner, so when its enabled again it starts normally
      if (!continuousPlanningParameters.getActiveMapping())
      {
         executionMode = ExecutionMode.OVERRIDE;
         continuousPlanner.setInitialized(false);
         multiplierForGoalPoseDistance = 1;
         return;
      }

      // Initialize the continuous planner so that the state machine starts in the correct configuration
      if (!continuousPlanner.isInitialized())
      {
         initializeContinuousPlanner();
         getImminentStanceFromLatestStatus();
         startPoseForFootstepPlanner = continuousPlanner.updateimminentStance(firstImminentFootstep, secondImminentFootstep, secondImminentFootstepSide);
         ActiveMappingTools.setStraightGoalPoses(originalReferenceFrameToBaseGoalPoseDirectionFrom,
                                                 multiplierForGoalPoseDistance, originalPoseToBaseGoalPoseFrom, startPoseForFootstepPlanner,
                                                 goalPoseForFootstepPlanner, 0.8f);
         plannerOutput = continuousPlanner.updatePlan(latestHeightMapData, startPoseForFootstepPlanner, goalPoseForFootstepPlanner, secondImminentFootstepSide);
         if (continuousPlanner.getFootstepPlanningResult() != FootstepPlanningResult.INVALID_GOAL)
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;
         else
            continuousPlannerState = ContinuousWalkingState.PLANNING_FAILED;
      }
      else if (!continuousPlanningParameters.getPauseContinuousWalking())
      {
         // The state machine will always run this method if the continuous planner is initialized and not paused
         System.out.println("----- Do Continuous Planning Loop -----");
         planAndSendFootsteps();
      }
   }

   public void planAndSendFootsteps()
   {
      // A foot is in swing, plan another step
      if ((footstepStatusMessage.get().getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED
          && continuousPlannerState != ContinuousWalkingState.FOOTSTEP_STARTED) || continuousPlannerState == ContinuousWalkingState.PLANNING_FAILED)
      {
         getImminentStanceFromLatestStatus();
         startPoseForFootstepPlanner = continuousPlanner.updateimminentStance(firstImminentFootstep, secondImminentFootstep, secondImminentFootstepSide);

         // TODO adjust this so that it doesn't consider the Z of the poses at all, otherwise there will be bugs in the future
         // Only update the goal poses if the robot gets within some distance of them
         double distance = firstImminentFootstep.getPositionDistance(goalPoseForFootstepPlanner.get(secondImminentFootstepSide.getOppositeSide()));
         if (distance < 0.46)
         {
            multiplierForGoalPoseDistance += 1;
            ActiveMappingTools.setStraightGoalPoses(originalReferenceFrameToBaseGoalPoseDirectionFrom,
                                                    multiplierForGoalPoseDistance, originalPoseToBaseGoalPoseFrom, startPoseForFootstepPlanner,
                                                    goalPoseForFootstepPlanner, 0.6f);
         }

         plannerOutput = continuousPlanner.updatePlan(latestHeightMapData, startPoseForFootstepPlanner, goalPoseForFootstepPlanner, secondImminentFootstepSide);

         if (continuousPlanner.getFootstepPlanningResult() != FootstepPlanningResult.INVALID_GOAL)
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;
         else
            continuousPlannerState = ContinuousWalkingState.PLANNING_FAILED;
      }
      // If we got a plan from the last time the planner ran, and we are running low on footsteps in the queue, send this plan
      else if (continuousPlanner.isPlanAvailable() && continuousPlannerState == ContinuousWalkingState.FOOTSTEP_STARTED
               && queuedFootstepSize < MAXIMUM_FOOTSTEPS_HELD_IN_CONTROLLER_QUEUE)
      {
         FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(plannerOutput,
                                                                                                        MAXIMUM_FOOTSTEPS_TO_SEND,
                                                                                                        SWING_DURATION,
                                                                                                        TRANSFER_DURATION);
         footstepDataList.getQueueingProperties().setExecutionMode(executionMode.toByte());

         if (originalFootstepDataListId == -1)
            originalFootstepDataListId = footstepDataList.getUniqueId();
         footstepDataList.setUniqueId(originalFootstepDataListId);

         LogTools.info("Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller");
         publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

         continuousPlannerState = ContinuousWalkingState.NOT_STARTED;
         continuousPlanner.setPlanAvailable(false);
         executionMode = ExecutionMode.QUEUE;
      }
   }

   /**
    * This method gets called when we need a new footstep plan, this gets the latest information in the controller footstep queue
    */
   public void getImminentStanceFromLatestStatus()
   {
      LogTools.info("Controller Queue Size used for planning: " + queuedFootstepSize);

      // Both imminent footsteps will be from the queue
      if (queuedFootstepSize > 1)
      {
         secondImminentFootstepSide = RobotSide.fromByte(queuedFootstepList.get(queuedFootstepSize - 1).getRobotSide());

         firstImminentFootstep.getTranslation().set(queuedFootstepList.get(queuedFootstepSize - 2).getLocation());
         firstImminentFootstep.getOrientation().set(queuedFootstepList.get(queuedFootstepSize - 2).getOrientation());

         secondImminentFootstep.getTranslation().set(queuedFootstepList.get(queuedFootstepSize - 1).getLocation());
         secondImminentFootstep.getOrientation().set(queuedFootstepList.get(queuedFootstepSize - 1).getOrientation());
      }
      // One imminent footstep will be from the queue, the other one will be the robot's real foot
      else if (queuedFootstepSize == 1)
      {
         secondImminentFootstepSide = RobotSide.fromByte(queuedFootstepList.get(0).getRobotSide());
         firstImminentFootstep.setFromReferenceFrame(referenceFrames.getSoleFrame(secondImminentFootstepSide.getOppositeSide()));

         secondImminentFootstep.getTranslation().set(queuedFootstepList.get(0).getLocation());
         secondImminentFootstep.getOrientation().set(queuedFootstepList.get(0).getOrientation());
      }
      // Both imminent footsteps will be the robot's real feet
      else if (queuedFootstepSize == 0)
      {
         secondImminentFootstepSide = RobotSide.RIGHT;
         firstImminentFootstep.setFromReferenceFrame(referenceFrames.getSoleFrame(secondImminentFootstepSide.getOppositeSide()));
         secondImminentFootstep.setFromReferenceFrame(referenceFrames.getSoleFrame(secondImminentFootstepSide));
      }
   }

   public SideDependentList<FramePose3D> getGoalPoseForFootstepPlanner()
   {
      return goalPoseForFootstepPlanner;
   }

   public SideDependentList<FramePose3D> getStartPoseForFootstepPlanner()
   {
      return startPoseForFootstepPlanner;
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      if (queuedFootstepSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
         LogTools.warn("Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
      queuedFootstepSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
      queuedFootstepList = footstepQueueStatusMessage.getQueuedFootstepList();
   }

   public void onHeightMapReceived(ImageMessage imageMessage)
   {
        if (heightMapImage == null)
        {
           int compressedBufferDefaultSize = 100000;
           heightMapImage = new Mat(imageMessage.getImageHeight(), imageMessage.getImageWidth(), opencv_core.CV_16UC1);
           compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
           incomingCompressedImageBuffer = NativeMemoryTools.allocate(compressedBufferDefaultSize);
           incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);
           LogTools.warn("Creating Buffer of Size: {}", compressedBufferDefaultSize);
        }

        PerceptionMessageTools.convertToHeightMapImage(imageMessage,
                                                       heightMapImage,
                                                       incomingCompressedImageBuffer,
                                                       incomingCompressedImageBytePointer,
                                                       compressedBytesMat);

        if (latestHeightMapData == null)
        {
           latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS,
                                                   RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                   imageMessage.getPosition().getX(),
                                                   imageMessage.getPosition().getY());
        }
        PerceptionMessageTools.convertToHeightMapData(heightMapImage,
                                                      latestHeightMapData,
                                                      imageMessage.getPosition(),
                                                      RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                      RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS);
   }

   public void setLatestHeightMapData(HeightMapData latestHeightMapData)
   {
      this.latestHeightMapData = new HeightMapData(latestHeightMapData);
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
