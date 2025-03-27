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
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
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
import us.ihmc.sensors.realsense.RealSenseDevice;

import java.time.Instant;
import java.util.List;

/**
 * This class takes care of managing the {@link RapidHeightMapExtractorCUDA}. This class can be used on remote process's
 * or locally as well.
 */
public class RapidHeightMapManager
{
   private final RapidHeightMapExtractorCUDA rapidHeightMapExtractor;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final Mat hostDepthImage = new Mat();
   private final Notification resetHeightMapRequested = new Notification();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;
   private final CUDAFlyingPointsFilter flyingPointsFilter;

   private final GpuMat deviceDepthImage;
   private final HeightMapParameters heightMapParameters;
   private final ROS2Publisher<ImageMessage> heightMapPublisher;
   private List<Collidable> robotCollidables;
   private CUDABodyCollisionFilter bodyCollisionFilter;
   private CameraIntrinsics depthImageIntrinsics;

   public RapidHeightMapManager(ROS2Node ros2Node,
                                RobotCollisionModel robotCollisionModel,
                                FullHumanoidRobotModel robotModel,
                                String robotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                CameraIntrinsics depthImageIntrinsics,
                                HeightMapParameters heightMapParameters,
                                ReferenceFrame cameraFrame) throws Exception
   {
      this.heightMapParameters = heightMapParameters;
      this.depthImageIntrinsics = depthImageIntrinsics;

      robotCollidables = robotCollisionModel.getRobotCollidables(robotModel.getRootBody());
      bodyCollisionFilter = new CUDABodyCollisionFilter();

      deviceDepthImage = new GpuMat(depthImageIntrinsics.getHeight(),
                                    depthImageIntrinsics.getWidth(),
                                    opencv_core.CV_16UC1);
      rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(leftFootSoleFrame,
                                                                rightFootSoleFrame,
                                                                deviceDepthImage,
                                                                depthImageIntrinsics,
                                                                1,
                                                                heightMapParameters);
      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
      flyingPointsFilter = new CUDAFlyingPointsFilter();

      heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_CROPPED);

      // We use a notification in order to only call resetting the height map in one place
      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      if (robotModel != null)
      {
         ros2Node.createSubscription(HumanoidControllerAPI.getTopic(HighLevelStateChangeStatusMessage.class, robotName),
                                     message ->
                                     {
                                        if (message.takeNextData().getEndHighLevelControllerName()
                                            == HighLevelStateChangeStatusMessage.WALKING)
                                        {
                                           resetHeightMapRequested.set();
                                        }
                                     });
      }
   }

   public void update(Mat latestDepthImage,
                      Instant imageAcquisitionTime,
                      ReferenceFrame cameraFrame,
                      ReferenceFrame cameraZUpFrame) throws Exception
   {
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
      {
         OpenCVTools.convertFloatToShort(latestDepthImage, hostDepthImage, 1000.0, 0.0);
      }
      else
      {
         latestDepthImage.convertTo(hostDepthImage, opencv_core.CV_16UC1);
      }

      deviceDepthImage.upload(hostDepthImage);
      bodyCollisionFilter.process(latestDepthImage,depthImageIntrinsics,robotCollidables, cameraFrame);
      if (resetHeightMapRequested.poll())
      {
         rapidHeightMapExtractor.reset();
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      if (heightMapParameters.getDriftOffsetFilter())
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(driftOffsetInZ);
         }
      }
      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, groundToWorld);

      Mat croppedHeightMapImage = rapidHeightMapExtractor.getVisualizedHeightMap();

      if (heightMapParameters.getFlyingPointsFilter())
      {
         GpuMat deviceCroppedHeightMapImage = new GpuMat();
         deviceCroppedHeightMapImage.upload(croppedHeightMapImage);
         GpuMat filteredDeviceCroppedHeightMapImage = flyingPointsFilter.applyFilter(deviceCroppedHeightMapImage);
         filteredDeviceCroppedHeightMapImage.download(croppedHeightMapImage);
         filteredDeviceCroppedHeightMapImage.close();
         deviceCroppedHeightMapImage.close();
      }

      OpenCVTools.compressImagePNG(croppedHeightMapImage, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         croppedHeightMapImageMessage,
                                                         heightMapPublisher,
                                                         cameraPose,
                                                         imageAcquisitionTime,
                                                         rapidHeightMapExtractor.getSequenceNumber(),
                                                         croppedHeightMapImage.rows(),
                                                         croppedHeightMapImage.cols(),
                                                         (float) heightMapParameters.getHeightScaleFactor());
   }

   public HeightMapData getLatestHeightMapData()
   {
      return rapidHeightMapExtractor.getHeightMapData();
   }

   public TerrainMapData getTerrainMapData()
   {
      return rapidHeightMapExtractor.getTerrainMapData();
   }

   public void destroy()
   {
      rapidHeightMapExtractor.destroy();
      flyingPointsFilter.destroy();
   }
}