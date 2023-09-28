package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.LocalizationAndMappingTask;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousMappingRemoteTask
{
   private final static long PLANNING_PERIOD_MS = 2000;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                   getClass(),
                                                                   ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>(new WalkingStatusMessage());
   private final ROS2PublisherMap publisherMap;
   private final ROS2Topic controllerFootstepDataTopic;

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;

   private int compressedBufferDefaultSize = 100000;

   private ActiveMapper activeMappingModule;
   private ROS2Helper ros2Helper;

   public ContinuousMappingRemoteTask(DRCRobotModel robotModel,
                                      ROS2Node ros2Node,
                                      HumanoidReferenceFrames referenceFrames)
   {
      this.walkingStatusMessage.get().setWalkingStatus(WalkingStatus.COMPLETED.toByte());
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      activeMappingModule = new ActiveMapper(robotModel, referenceFrames, ActiveMapper.ActiveMappingMode.CONTINUOUS_MAPPING_COVERAGE);
      ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), this::walkingStatusReceived);

      executorService.scheduleAtFixedRate(this::updateActiveMappingPlan,0, PLANNING_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   private void walkingStatusReceived(WalkingStatusMessage walkingStatusMessage)
   {
      LogTools.warn("Received Walking Status Message: {}", walkingStatusMessage);
      this.walkingStatusMessage.set(walkingStatusMessage);
   }

   /**
    * Scheduled on regular intervals to compute and send the active mapping plan to the controller.
    */
   private void updateActiveMappingPlan()
   {
      if (walkingStatusMessage.get() != null)
      {
         activeMappingModule.setWalkingStatus(WalkingStatus.fromByte(walkingStatusMessage.get().getWalkingStatus()));
         if (walkingStatusMessage.get().getWalkingStatus() == WalkingStatusMessage.COMPLETED && !activeMappingModule.isPlanAvailable())
         {
            activeMappingModule.updatePlan(latestHeightMapData);
         }
      }

      if (activeMappingModule.isPlanAvailable())
      {
         FootstepDataListMessage footstepDataList = activeMappingModule.getFootstepDataListMessage();
         publisherMap.publish(controllerFootstepDataTopic, footstepDataList);
         activeMappingModule.setPlanAvailable(false);
      }
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

        if (latestHeightMapData == null)
        {
           latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS,
                                                   RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                   imageMessage.getPosition().getX(),
                                                   imageMessage.getPosition().getY());
        }
        PerceptionMessageTools.convertToHeightMapData(heightMapImage.ptr(0),
                                                      latestHeightMapData,
                                                      imageMessage.getPosition(),
                                                      RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                      RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS);
   }

   public ActiveMapper getActiveMappingModule()
   {
      return activeMappingModule;
   }

   public void destroy()
   {
      executorService.shutdown();
   }
}
