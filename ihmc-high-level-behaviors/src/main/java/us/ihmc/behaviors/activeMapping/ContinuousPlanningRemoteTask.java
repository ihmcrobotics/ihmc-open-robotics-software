package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.ByteBuffer;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlanningRemoteTask
{
   private final static long CONTINUOUS_PLANNING_UPDATE_TICK_MS = 20;
   private final static float SWING_DURATION = 0.8f;
   private final static float TRANSFER_DURATION = 0.4f;
   private final static int MAXIMUM_FOOTSTEPS_TO_SEND = 1;

   private enum ContinuousWalkingState
   {
      NOT_STARTED,
      INITIALIZED,
      FOOTSTEP_STARTED
   }

   private ContinuousWalkingState continuousPlannerState = ContinuousWalkingState.NOT_STARTED;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                   getClass(),
                                                                   ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());

   private int queuedFootstepSize;

   private final ROS2PublisherMap publisherMap;
   private final ROS2Topic controllerFootstepDataTopic;

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private final PerceptionConfigurationParameters perceptionConfigurationParameters;

   private int compressedBufferDefaultSize = 100000;

   private final HumanoidReferenceFrames referenceFrames;

   private final ContinuousPlanner continuousPlanner;
   private FootstepPlannerOutput plannerOutput;
   private final SideDependentList<FramePose3D> goalPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private RobotSide initialStanceSide = RobotSide.LEFT;

   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private long originalFootstepDataListId = -1;

   public ContinuousPlanningRemoteTask(DRCRobotModel robotModel,
                                       ROS2Node ros2Node,
                                       HumanoidReferenceFrames referenceFrames,
                                       PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.referenceFrames = referenceFrames;
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, ContinuousPlanner.PlanningMode.MAX_COVERAGE);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepQueueStatusReceived);

      executorService.scheduleAtFixedRate(this::updateContinuousPlanner, 0, CONTINUOUS_PLANNING_UPDATE_TICK_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Runs the continuous mapper state machine every ACTIVE_MAPPING_UPDATE_TICK_MS milliseconds. The status messages decide the state of the machine.
    */
   private void updateContinuousPlanner()
   {
      if (perceptionConfigurationParameters.getActiveMapping())
      {
         if (!continuousPlanner.isInitialized()) // Initialize the active mapper footstep plan so that the state machine starts in the correct configuration
         {
            continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;

            initialize();
            ActiveMappingTools.setStraightGoalPoses(startPose, goalPose, 0.5f);
            plannerOutput = continuousPlanner.updatePlan(latestHeightMapData, startPose, goalPose, initialStanceSide); // Returns if planning in progress, sets planAvailable if plan was found
            continuousPlanner.setInitialized(true);
         }
         else // Initialized, so we can run the state machine in normal mode to eternity
         {
            if (footstepStatusMessage.get().getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED
                && continuousPlannerState != ContinuousWalkingState.FOOTSTEP_STARTED) // start planning, swing has started
            {
               continuousPlannerState = ContinuousWalkingState.FOOTSTEP_STARTED;

               ActiveMappingTools.setStraightGoalPoses(startPose, goalPose, 0.5f);
               plannerOutput = continuousPlanner.updatePlan(latestHeightMapData, startPose, goalPose, initialStanceSide);
            }
            // Only send the next footstep if a plan is available
            else if (continuousPlanner.isPlanAvailable() && continuousPlannerState == ContinuousWalkingState.FOOTSTEP_STARTED && queuedFootstepSize < 1)
            {
               continuousPlannerState = ContinuousWalkingState.NOT_STARTED;

               FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(plannerOutput,
                                                                                                              MAXIMUM_FOOTSTEPS_TO_SEND,
                                                                                                              SWING_DURATION,
                                                                                                              TRANSFER_DURATION);
               footstepDataList.getQueueingProperties().setExecutionMode(executionMode.toByte());

               if (originalFootstepDataListId == -1)
                  originalFootstepDataListId = footstepDataList.getUniqueId();

               footstepDataList.setUniqueId(originalFootstepDataListId);
               publisherMap.publish(controllerFootstepDataTopic, footstepDataList); // send it to the controller
               initialStanceSide = RobotSide.fromByte(footstepDataList.getFootstepDataList().get(0).getRobotSide());
               startPose = continuousPlanner.updateStanceAndSwitchSides(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                                        footstepDataList.getFootstepDataList().get(0).getLocation(),
                                                                                        footstepDataList.getFootstepDataList().get(0).getOrientation()),
                                                                        initialStanceSide);
               continuousPlanner.setPlanAvailable(false);
               executionMode = ExecutionMode.QUEUE;
            }
         }
      }
   }

   public void initialize()
   {
      startPose.get(RobotSide.LEFT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.LEFT));
      startPose.get(RobotSide.RIGHT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT));
   }

   public SideDependentList<FramePose3D> getGoalPose()
   {
      return goalPose;
   }

   public SideDependentList<FramePose3D> getStartPose()
   {
      return startPose;
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.warn("Received Footstep Status Message: {}", footstepStatusMessage);
      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      queuedFootstepSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   public void onHeightMapReceived(ImageMessage imageMessage)
   {
        if (heightMapImage == null)
        {
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
        zUpToWorldTransform.set(imageMessage.getOrientation(), imageMessage.getPosition());

        PerceptionDebugTools.displayHeightMap("Height Map", heightMapImage, 1, 1.2f);

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
      this.latestHeightMapData = latestHeightMapData;
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
