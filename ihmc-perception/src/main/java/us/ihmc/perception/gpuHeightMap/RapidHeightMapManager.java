package us.ihmc.perception.gpuHeightMap;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.bytedeco.javacpp.BytePointer;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDABodyCollisionFilter;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.filters.DepthImageFlyingPointsFilter;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.sensorProcessing.filters.DepthImageFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

/**
 * This class takes care of managing the {@link RapidHeightMapExtractorCUDA}. This class can be used in a remote process, or locally as well.
 */
public class RapidHeightMapManager
{
   private final HeightMapParameters heightMapParameters;
   private final RapidHeightMapExtractorCUDA rapidHeightMapExtractor;

   private final Point3D sensorOrigin = new Point3D();
   private final FramePose3D cameraPose = new FramePose3D();
   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final CUDABodyCollisionFilter bodyCollisionFilter;
   private final DepthImageFlyingPointsFilter flyingPointsFilter;
   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;

   private final ROS2Publisher<ImageMessage> heightMapPublisher;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();
   private final GpuMat deviceCroppedHeightMap;
   private final ROS2Publisher<ImageMessage> filteredDepthPublisher;

   public RapidHeightMapManager(ROS2Node ros2Node,
                                RobotCollisionModel robotCollisionModel,
                                FullHumanoidRobotModel robotModel,
                                String robotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                HeightMapParameters heightMapParameters,
                                DepthImageFilterParameters depthImageFilterParameters)
   {
      this.heightMapParameters = heightMapParameters;

      footSoleFrames.add(leftFootSoleFrame);
      footSoleFrames.add(rightFootSoleFrame);

      List<Collidable> robotCollidables = robotCollisionModel.getRobotCollidables(robotModel.getRootBody());
      bodyCollisionFilter = new CUDABodyCollisionFilter(robotCollidables);
      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
      flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilterParameters);

      int croppedCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getCroppedWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      int cellsPerAxisCropped = 2 * croppedCenterIndex + 1;
      deviceCroppedHeightMap = new GpuMat(cellsPerAxisCropped, cellsPerAxisCropped, opencv_core.CV_16UC1);
      rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(1, heightMapParameters);

      // We use a notification to only call resetting the height map in one place
      heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_CROPPED);
      filteredDepthPublisher = ros2Node.createPublisher(PerceptionAPI.D455_DEPTH_FILTERED_IMAGE);

      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      ros2Node.createSubscription2(PerceptionAPI.LOWER_HEIGHT_MAP_BACKDROP, message -> lowerHeightMapBackdropRequested.set());

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
   int debug = 0;

   public void updateAndPublishHeightMap(RawImage depthImage, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      RawImage depthImageCopy = depthImage.get();
      Mat latestDepthImage = depthImageCopy.getCpuImageMat();
      Instant acquisitionTime = depthImageCopy.getAcquisitionTime();
      CameraIntrinsics depthIntrinsicsCopy = depthImageCopy.getIntrinsicsCopy();

      // -------- Update the Height Map with the latest depth image from the sensor --------------
      updateInternal(latestDepthImage, deviceCroppedHeightMap, depthIntrinsicsCopy, cameraFrame, cameraZUpFrame);

      // Publish the height map to anyone who is subscribing
      Mat hostCroppedHeightMap = new Mat();
      deviceCroppedHeightMap.download(hostCroppedHeightMap);
      OpenCVTools.compressImagePNG(hostCroppedHeightMap, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         croppedHeightMapImageMessage,
                                                         heightMapPublisher,
                                                         cameraPose,
                                                         acquisitionTime,
                                                         rapidHeightMapExtractor.getSequenceNumber(),
                                                         hostCroppedHeightMap.rows(),
                                                         hostCroppedHeightMap.cols(),
                                                         (float) heightMapParameters.getHeightScaleFactor());

      hostCroppedHeightMap.close();
      depthImageCopy.release();
   }

   CUDACompressionTools cudaCompressionTools = new CUDACompressionTools();

   /**
    * Update the Height Map with the latest depth image from the sensor
    */
   private void updateInternal(RawImage rawImage,
                               Mat latestDepthImage,
                               GpuMat deviceHeightMapToPack,
                               CameraIntrinsics depthIntrinsicsCopy,
                               ReferenceFrame cameraFrame,
                               ReferenceFrame cameraZUpFrame)
   {
      // Option that gets triggered from a message sent from the user
      if (lowerHeightMapBackdropRequested.poll())
      {
         double footHeight = computeFootHeight();
         int loweredFootHeight = 10000;
         rapidHeightMapExtractor.reset(footHeight, loweredFootHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // Option that gets triggered from a message sent from the user
      if (resetHeightMapRequested.poll())
      {
         double footHeight = computeFootHeight();
         rapidHeightMapExtractor.reset(footHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // -------- Update the Height Map with the latest depth image from the sensor --------------
      // This takes the latest depth image and converts it to the expected type
      // Because we expect to run a lot of kernels on the GPU, convert to GpuMat until finished with kernels
      GpuMat deviceDepthImage = new GpuMat(latestDepthImage); // We are extremely prudent to close these to avoid memory leaks
      latestDepthImage.convertTo(deviceDepthImage, opencv_core.CV_16UC1);

      // Perform a flying points filter as a post-processing step on the height map
      if (heightMapParameters.getFlyingPointsFilter())
      {
         GpuMat deviceOutputImage = new GpuMat(deviceDepthImage.size(), deviceDepthImage.type());
         flyingPointsFilter.applyFilter(deviceDepthImage, deviceOutputImage, depthIntrinsicsCopy);
         deviceOutputImage.copyTo(deviceDepthImage);
         deviceOutputImage.close();

         BytePointer bytePointer = cudaCompressionTools.compressDepth(deviceDepthImage);

         ImageMessage imageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(rawImage, bytePointer, CompressionType.ZSTD_NVJPEG_HYBRID, imageMessage);
         filteredDepthPublisher.publish(imageMessage);
      }


      // We expect that depthImage to contain depths for parts of the robot that are in the camera frame, we remove that here
      GpuMat depthImageWithoutRobot = new GpuMat(deviceDepthImage.size(), deviceDepthImage.type());
      bodyCollisionFilter.process(deviceDepthImage, depthImageWithoutRobot, depthIntrinsicsCopy, cameraFrame);
      deviceDepthImage.close();   // Now we are finished with the device depth image because we have a new image without the robot, so close it


      // The controller can publish a status letting anyone listening know that the controller is aware of some amount of drift in the Z direction
      // If we have that parameter set to true, we update the heights of the height map to account for that drift
      if (heightMapParameters.getDriftOffsetFilter())
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(driftOffsetInZ, depthIntrinsicsCopy, sensorOrigin, computeFootHeight());
         }
      }

      // We expect to have knowledge of where the camera is in relation to the world so we can accurately display the height map
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();
      sensorOrigin.set(sensorToWorld.getTranslation());
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      // Perform update, this actually creates the height map
      rapidHeightMapExtractor.update(depthImageWithoutRobot,
                                     depthIntrinsicsCopy,
                                     sensorToWorld,
                                     sensorToGround,
                                     groundToWorld,
                                     sensorOrigin,
                                     computeFootHeight());

      GpuMat deviceCroppedHeightMap = rapidHeightMapExtractor.getCroppedHeightMap();
      // We have used the depth image without the robot, close this to avoid creating a memory leak
      depthImageWithoutRobot.close();

      // Don't close this mat as its being used in the extractor till that finish's
      deviceCroppedHeightMap.convertTo(deviceHeightMapToPack, deviceCroppedHeightMap.type());
   }

   public HeightMapData getLatestHeightMapData()
   {
      HeightMapData latestHeightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                                            (float) heightMapParameters.getCroppedWidthInMeters(),
                                                            sensorOrigin.getX(),
                                                            sensorOrigin.getY());

      Mat heightMap = new Mat();
      deviceCroppedHeightMap.download(heightMap);
      PerceptionMessageTools.convertToHeightMapData(heightMap,
                                                    latestHeightMapData,
                                                    sensorOrigin,
                                                    (float) heightMapParameters.getCroppedWidthInMeters(),
                                                    (float) heightMapParameters.getCellSizeInMeters(),
                                                    heightMapParameters);
      heightMap.close();

      return latestHeightMapData;
   }

   private double computeFootHeight()
   {
      double thicknessOfTheFoot = 0.02;
      double height = Double.POSITIVE_INFINITY;

      for (int i = 0; i < footSoleFrames.size(); i++)
      {
         height = Math.min(footSoleFrames.get(i).getTransformToWorldFrame().getTranslationZ(), height);
      }
      if (Double.isInfinite(height))
         height = 0.0;

      height -= thicknessOfTheFoot;

      return height;
   }

   public void destroy()
   {
      cudaCompressionTools.destroy();
      rapidHeightMapExtractor.destroy();
      flyingPointsFilter.destroy();
      deviceCroppedHeightMap.close();
      compressedCroppedHeightMapPointer.close();
   }
}