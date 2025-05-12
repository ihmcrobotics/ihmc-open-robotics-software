package us.ihmc.perception;

import com.vividsolutions.jts.geomgraph.Depth;
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
import us.ihmc.perception.filters.DepthImageFlyingPointsfilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;

public class RapidHeightMapThread extends RepeatingTaskThread
{
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;
   private final DepthImageFlyingPointsfilter flyingPointsfilter;
   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame cameraFrame;
   private final ReferenceFrame zUpSensorFrame;
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

      cameraFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();

      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);

      filteredDepthPublisher = ros2Node.createPublisher(PerceptionAPI.D455_DEPTH_FILTERED_IMAGE);

      bodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, syncedRobotModel.getFullRobotModel().getRootBody());
      flyingPointsfilter = new DepthImageFlyingPointsfilter(depthImageFilteringParameters);
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

         // Process body collisions
         GpuMat depthImageWithoutBodyCollisions = new GpuMat(latestDepthImage.size(), latestDepthImage.type());
         bodyCollisionFilter.process(latestDepthImage, depthImageWithoutBodyCollisions, depthIntrinsicsCopy, cameraFrame);

         GpuMat depthImageFiltered = new GpuMat(depthImageWithoutBodyCollisions.size(), depthImageWithoutBodyCollisions.type());
         flyingPointsfilter.applyFilter(depthImageWithoutBodyCollisions, depthImageFiltered, depthIntrinsicsCopy);
         depthImageWithoutBodyCollisions.close();

         BytePointer bytePointer = cudaCompressionTools.compressDepth(depthImageFiltered);

         ImageMessage imageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(depthImage, bytePointer, CompressionType.ZSTD_NVJPEG_HYBRID, imageMessage);
         filteredDepthPublisher.publish(imageMessage);

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.updateAndPublishHeightMap(depthImageFiltered, acquisitionTime, depthIntrinsicsCopy, cameraFrame, zUpSensorFrame);
         }

         depthImageFiltered.close();
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

      bodyCollisionFilter.close();
      flyingPointsfilter.destroy();
      heightMapManager.destroy();
   }
}
