package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
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
   private final static long CONTINUOUS_PLANNING_UPDATE_TICK_MS = 10;
   private final static float SWING_DURATION = 0.84f;
   private final static float TRANSFER_DURATION = 0.46f;
   private final static int MAXIMUM_FOOTSTEPS_TO_SEND = 1;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                   getClass(),
                                                                   ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>(new WalkingStatusMessage());
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<FootstepQueueStatusMessage> footstepQueueStatusMessage = new AtomicReference<>(new FootstepQueueStatusMessage());

   private final ROS2PublisherMap publisherMap;
   private final ROS2Topic controllerFootstepDataTopic;

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private PerceptionConfigurationParameters perceptionConfigurationParameters;

   private int compressedBufferDefaultSize = 100000;

   private ContinuousPlanner continuousPlanner;
   private ROS2Helper ros2Helper;

   private FootstepPlannerOutput footstepPlanner;

   public ContinuousPlanningRemoteTask(DRCRobotModel robotModel,
                                       ROS2Node ros2Node,
                                       HumanoidReferenceFrames referenceFrames,
                                       PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
      this.walkingStatusMessage.get().setWalkingStatus(WalkingStatus.COMPLETED.toByte());
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      continuousPlanner = new ContinuousPlanner(robotModel, referenceFrames, ContinuousPlanner.PlanningMode.MAX_COVERAGE);
      ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), this::walkingStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepStatusReceived);
      //ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepQueueStatusMessage.class, robotModel.getSimpleRobotName()), this::footstepQueueStatusReceived);


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
               continuousPlanner.initialize();
               footstepPlanner = continuousPlanner.updatePlan(latestHeightMapData); // Returns if planning in progress, sets planAvailable if plan was found
               continuousPlanner.setInitialized(true);
         }
         else // Initialized, so we can run the state machine in normal mode to eternity
         {
            if (footstepStatusMessage.get().getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED) // start planning, swing has started
            {
               footstepPlanner = continuousPlanner.updatePlan(latestHeightMapData);
            }
            else
            {
               if (continuousPlanner.isPlanAvailable()) // Only send the next footstep if a plan is available
               {
                  FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(footstepPlanner,
                                                                                                                 MAXIMUM_FOOTSTEPS_TO_SEND,
                                                                                                                 SWING_DURATION,
                                                                                                                 TRANSFER_DURATION);
                  publisherMap.publish(controllerFootstepDataTopic, footstepDataList); // send it to the controller
                  continuousPlanner.updateStanceAndSwitchSides(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                               footstepDataList.getFootstepDataList().get(0).getLocation(),
                                                                               footstepDataList.getFootstepDataList().get(0).getOrientation()),
                                                               RobotSide.fromByte(footstepDataList.getFootstepDataList().get(0).getRobotSide()));
                  continuousPlanner.setPlanAvailable(false);
               }
            }
         }
      }
   }

   private void walkingStatusReceived(WalkingStatusMessage walkingStatusMessage)
   {
      LogTools.warn("Received Walking Status Message: {}", walkingStatusMessage);
      this.walkingStatusMessage.set(walkingStatusMessage);
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.warn("Received Footstep Status Message: {}", footstepStatusMessage);
      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      LogTools.warn("Received Footstep Queue Status Message: {}", footstepQueueStatusMessage);
      this.footstepQueueStatusMessage.set(footstepQueueStatusMessage);
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
