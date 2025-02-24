package us.ihmc.perception.gpuHeightMap;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.filters.CUDAFlyingPointsFilter;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.time.Instant;

/**
 * This class takes care of managing a {@link RapidHeightMapExtractor}. This class can be used on remote process's or locally as well.
 */
public class RapidHeightMapManager
{
   static final HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");

   private final RapidHeightMapExtractorInterface rapidHeightMapExtractor;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Node ros2Node;
   private final boolean runWithCUDA;
   private final Mat hostDepthImage = new Mat();
   private final Notification resetHeightMapRequested = new Notification();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   private RapidHeightMapDriftOffset rapidHeightMapDriftOffset;
   private CUDAFlyingPointsFilter flyingPointsFilter;

   private GpuMat deviceDepthImage;
   private BytedecoImage heightMapBytedecoImage;
   private ROS2Publisher<ImageMessage> heightMapPublisher;

   public RapidHeightMapManager(ROS2Node ros2Node,
                                FullHumanoidRobotModel robotModel,
                                String robotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                CameraIntrinsics depthImageIntrinsics,
                                boolean runWithCUDA) throws Exception
   {
      this.ros2Node = ros2Node;
      this.runWithCUDA = runWithCUDA;

      if (runWithCUDA)
      {
         deviceDepthImage = new GpuMat(depthImageIntrinsics.getHeight(), depthImageIntrinsics.getWidth(), opencv_core.CV_16UC1);
         rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(leftFootSoleFrame,
                                                                   rightFootSoleFrame,
                                                                   deviceDepthImage,
                                                                   depthImageIntrinsics,
                                                                   1,
                                                                   heightMapParameters);
         rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
         flyingPointsFilter = new CUDAFlyingPointsFilter();
      }
      else
      {
         OpenCLManager openCLManager = new OpenCLManager();
         heightMapBytedecoImage = new BytedecoImage(depthImageIntrinsics.getWidth(), depthImageIntrinsics.getHeight(), opencv_core.CV_16UC1);
         heightMapBytedecoImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         rapidHeightMapExtractor = new RapidHeightMapExtractor(openCLManager,
                                                               leftFootSoleFrame,
                                                               rightFootSoleFrame,
                                                               heightMapBytedecoImage,
                                                               depthImageIntrinsics,
                                                               1,
                                                               heightMapParameters);
      }

      // We use a notification in order to only call resetting the height map in one place
      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      if (robotModel != null)
      {
         ros2Node.createSubscription(HumanoidControllerAPI.getTopic(HighLevelStateChangeStatusMessage.class, robotName), message ->
         {
            if (message.takeNextData().getEndHighLevelControllerName() == HighLevelStateChangeStatusMessage.WALKING)
            {
               resetHeightMapRequested.set();
            }
         });
      }
   }

   public void update(Mat latestDepthImage, Instant imageAcquisitionTime, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame) throws Exception
   {
      if (runWithCUDA)
      {
         if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
         {
            OpenCVTools.convertFloatToShort(latestDepthImage, hostDepthImage, 1000.0, 0.0);
         }
         else
         {
            latestDepthImage.convertTo(hostDepthImage, opencv_core.CV_16UC1);
         }

         deviceDepthImage.upload(hostDepthImage);
      }
      else
      {
         if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
         {
            OpenCVTools.convertFloatToShort(latestDepthImage, heightMapBytedecoImage.getBytedecoOpenCVMat(), 1000.0, 0.0);
         }
         else
         {
            latestDepthImage.convertTo(heightMapBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1);
         }
      }

      if (resetHeightMapRequested.poll())
      {
         rapidHeightMapExtractor.reset();
         if (rapidHeightMapDriftOffset != null)
            rapidHeightMapDriftOffset.reset();
      }

      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      if (runWithCUDA)
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(driftOffsetInZ);
         }
      }

      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, groundToWorld);

      Mat croppedHeightMapImage = rapidHeightMapExtractor.getTerrainMapData().getHeightMap();

      if (runWithCUDA && getHeightMapParameters().getFlyingPointsFilter())
      {
         GpuMat deviceCroppedHeightMapImage = new GpuMat();
         deviceCroppedHeightMapImage.upload(croppedHeightMapImage);
         GpuMat filteredDeviceCroppedHeightMapImage = flyingPointsFilter.applyFilter(deviceCroppedHeightMapImage);
         filteredDeviceCroppedHeightMapImage.download(croppedHeightMapImage);
         filteredDeviceCroppedHeightMapImage.close();
         deviceCroppedHeightMapImage.close();
      }

      if (heightMapPublisher == null)
      {
         heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_CROPPED);
      }

      OpenCVTools.compressImagePNG(croppedHeightMapImage, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         PerceptionAPI.HEIGHT_MAP_CROPPED,
                                                         croppedHeightMapImageMessage,
                                                         heightMapPublisher,
                                                         cameraPose,
                                                         imageAcquisitionTime,
                                                         rapidHeightMapExtractor.getSequenceNumber(),
                                                         croppedHeightMapImage.rows(),
                                                         croppedHeightMapImage.cols(),
                                                         (float) getHeightMapParameters().getHeightScaleFactor());
   }

   public HeightMapData getLatestHeightMapData()
   {
      return rapidHeightMapExtractor.getHeightMapData();
   }

   public TerrainMapData getTerrainMapData()
   {
      return rapidHeightMapExtractor.getTerrainMapData();
   }

   public static HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }

   public void destroy()
   {
      rapidHeightMapExtractor.destroy();
      flyingPointsFilter.destroy();
   }
}