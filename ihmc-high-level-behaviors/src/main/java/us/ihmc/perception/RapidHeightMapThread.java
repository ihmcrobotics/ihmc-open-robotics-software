package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.filters.DepthImageBodyCollisionFilter;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.perception.filters.DepthImageFlyingPointsFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;

public class RapidHeightMapThread extends RepeatingTaskThread
{
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;
   private final DepthImageFlyingPointsFilter flyingPointsFilter;
   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame cameraFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final HeightMapParameters heightMapParameters;
   private final int depthImageKey;
   private final CUDACompressionTools cudaCompressionTools = new CUDACompressionTools();
   private final ROS2Publisher<ImageMessage> filteredDepthPublisher;

   public RapidHeightMapThread(ROS2Node ros2Node,
                               ROS2SyncedRobotModel syncedRobotModel,
                               RobotCollisionModel robotCollisionModel,
                               ImageSensor imageSensor,
                               int depthImageKey,
                               ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                               HeightMapParameters heightMapParameters,
                               DepthImageFilteringParameters depthImageFilteringParameters)
   {
      super(imageSensor.getSensorName() + RapidHeightMapThread.class.getSimpleName());

      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;
      this.heightMapParameters = heightMapParameters;

      cameraFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();

      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);

      filteredDepthPublisher = ros2Node.createPublisher(PerceptionAPI.D455_DEPTH_FILTERED_IMAGE);

      bodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, syncedRobotModel.getFullRobotModel().getRootBody());
      flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
      heightMapManager = new RapidHeightMapManager(ros2Node,
                                                   syncedRobotModel.getFullRobotModel(),
                                                   syncedRobotModel.getRobotModel().getSimpleRobotName(),
                                                   leftFootFrame,
                                                   rightFootFrame,
                                                   controllerFootstepQueueMonitor,
                                                   heightMapParameters);
   }

   @Override
   protected void runTask()
   {
      try
      {
         imageSensor.waitForGrab();
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Get everything we need from the image
         GpuMat latestDepthImage = depthImage.getGpuImageMat();
         Instant acquisitionTime = depthImage.getAcquisitionTime();
         CameraIntrinsics depthIntrinsicsCopy = depthImage.getIntrinsicsCopy();
         GpuMat filteredDepthImage = new GpuMat(latestDepthImage.size(), latestDepthImage.type());

         // Process body collisions
         bodyCollisionFilter.process(latestDepthImage, filteredDepthImage, depthIntrinsicsCopy, cameraFrame);

         if (heightMapParameters.getFlyingPointsFilter())
         {
            GpuMat depthImageNoFlyingPoints = new GpuMat(filteredDepthImage.size(), filteredDepthImage.type());
            flyingPointsFilter.applyFilter(filteredDepthImage, depthImageNoFlyingPoints, depthIntrinsicsCopy);
            depthImageNoFlyingPoints.copyTo(filteredDepthImage);

            BytePointer bytePointer = cudaCompressionTools.compressDepth(depthImageNoFlyingPoints);

            ImageMessage imageMessage = new ImageMessage();
            PerceptionMessageTools.packImageMessage(depthImage, bytePointer, CompressionType.ZSTD_NVJPEG_HYBRID, imageMessage);
            filteredDepthPublisher.publish(imageMessage);

            depthImageNoFlyingPoints.close();
         }

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.updateAndPublishHeightMap(filteredDepthImage, acquisitionTime, depthIntrinsicsCopy, cameraFrame, zUpSensorFrame);
         }

         filteredDepthImage.close();
         depthImage.release();
      }
      catch (InterruptedException ignored)
      {
      }
      catch (Exception e)
      {
         LogTools.error(e);
      }
   }

   public HeightMapData getLatestHeightMapData()
   {
      synchronized (heightMapLock)
      {
         return heightMapManager.getLatestHeightMapData();
      }
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      cudaCompressionTools.destroy();
      bodyCollisionFilter.close();
      flyingPointsFilter.destroy();
      heightMapManager.destroy();
   }
}
