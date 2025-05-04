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
import us.ihmc.perception.filters.CUDAFlyingPointsFilter;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

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
   private final SnappingTerrainExtractor snappedFootstepsExtractor;

   private final Point3D sensorOrigin = new Point3D();
   private final FramePose3D cameraPose = new FramePose3D();
   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final CUDABodyCollisionFilter bodyCollisionFilter;
   private final CUDAFlyingPointsFilter flyingPointsFilter;
   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;

   private final ROS2Publisher<ImageMessage> heightMapPublisher;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   public RapidHeightMapManager(ROS2Node ros2Node,
                                RobotCollisionModel robotCollisionModel,
                                FullHumanoidRobotModel robotModel,
                                String robotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;

      footSoleFrames.add(leftFootSoleFrame);
      footSoleFrames.add(rightFootSoleFrame);

      List<Collidable> robotCollidables = robotCollisionModel.getRobotCollidables(robotModel.getRootBody());
      bodyCollisionFilter = new CUDABodyCollisionFilter(robotCollidables);
      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
      flyingPointsFilter = new CUDAFlyingPointsFilter();

      rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(1, heightMapParameters);
      snappedFootstepsExtractor = new SnappingTerrainExtractor(heightMapParameters);

      // We use a notification to only call resetting the height map in one place
      heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_CROPPED);
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

   public void update(RawImage depthImage, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame) throws Exception
   {
      // -------- Update the Height Map with the latest depth image from the sensor --------------
      RawImage depthImageCopy = depthImage.get();
      Mat latestDepthImage = depthImageCopy.getCpuImageMat();
      Instant acquisitionTime = depthImageCopy.getAcquisitionTime();
      CameraIntrinsics depthIntrinsicsCopy = depthImageCopy.getIntrinsicsCopy();

      update(latestDepthImage, acquisitionTime, depthIntrinsicsCopy, cameraFrame, cameraZUpFrame);
      depthImageCopy.release();
   }

   public void update(Mat latestDepthImage,
                      Instant acquisitionTime,
                      CameraIntrinsics depthIntrinsicsCopy,
                      ReferenceFrame cameraFrame,
                      ReferenceFrame cameraZUpFrame) throws Exception
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
      GpuMat deviceCroppedHeightMap = rapidHeightMapExtractor.getVisualizedHeightMap();
      // We have used the depth image without the robot, close this to avoid creating a memory leak
      depthImageWithoutRobot.close();

      // Perform a flying points filter as a post-processing step on the height map
      if (heightMapParameters.getFlyingPointsFilter())
      {
         GpuMat deviceOutputImage = new GpuMat(deviceCroppedHeightMap.size(), deviceCroppedHeightMap.type());
         flyingPointsFilter.applyFilter(deviceCroppedHeightMap, deviceOutputImage);
         deviceOutputImage.copyTo(deviceCroppedHeightMap);
         deviceOutputImage.close();
      }

      // Now extract the maps to be used in the footstep planning algorithm
      snappedFootstepsExtractor.update(rapidHeightMapExtractor.getTerrainHeightMapImage(), sensorOrigin);

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
   }

   public TerrainMapData getTerrainMapData()
   {
      return snappedFootstepsExtractor.getTerrainMapData();
   }

   public HeightMapData getLatestHeightMapData()
   {
      HeightMapData latestHeightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                                            sensorOrigin.getX(),
                                                            sensorOrigin.getY());

      Mat heightMapMat = getTerrainMapData().getHeightMap();
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    sensorOrigin,
                                                    (float) heightMapParameters.getTerrainWidthInMeters(),
                                                    (float) heightMapParameters.getCellSizeInMeters(),
                                                    heightMapParameters);

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
      rapidHeightMapExtractor.destroy();
      snappedFootstepsExtractor.close();
      flyingPointsFilter.destroy();
   }
}