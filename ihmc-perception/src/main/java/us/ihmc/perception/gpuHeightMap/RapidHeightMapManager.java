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
import java.util.List;

/**
 * This class takes care of managing the {@link RapidHeightMapExtractorCUDA}. This class can be used in a remote process, or locally as well.
 */
public class RapidHeightMapManager
{
   private final RapidHeightMapExtractorCUDA rapidHeightMapExtractor;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;
   private final CUDAFlyingPointsFilter flyingPointsFilter;

   private final HeightMapParameters heightMapParameters;
   private final ROS2Publisher<ImageMessage> heightMapPublisher;
   private final CUDABodyCollisionFilter bodyCollisionFilter;

   public RapidHeightMapManager(ROS2Node ros2Node,
                                RobotCollisionModel robotCollisionModel,
                                FullHumanoidRobotModel robotModel,
                                String robotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                HeightMapParameters heightMapParameters) throws Exception
   {
      this.heightMapParameters = heightMapParameters;

      List<Collidable> robotCollidables = robotCollisionModel.getRobotCollidables(robotModel.getRootBody());
      bodyCollisionFilter = new CUDABodyCollisionFilter(robotCollidables);

      rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(leftFootSoleFrame, rightFootSoleFrame, 1, heightMapParameters);
      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
      flyingPointsFilter = new CUDAFlyingPointsFilter();

      heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_CROPPED);

      // We use a notification to only call resetting the height map in one place
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
      // Option that gets triggered from a message sent from the user
      if (lowerHeightMapBackdropRequested.poll())
      {
         rapidHeightMapExtractor.lowerBackDrop();
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // Option that gets triggered from a message sent from the user
      if (resetHeightMapRequested.poll())
      {
         rapidHeightMapExtractor.reset();
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // -------- Update the Height Map with the latest depth image from the sensor --------------
      Instant acquisitionTime = depthImage.getAcquisitionTime();
      Mat latestDepthImage = depthImage.getCpuImageMat();
      CameraIntrinsics depthIntrinsicsCopy = depthImage.getIntrinsicsCopy();
      // This takes the latest depth image and converts it to the expected type
      Mat hostDepthImage = depthImage.getCpuImageMat();
      latestDepthImage.convertTo(hostDepthImage, opencv_core.CV_16UC1);

      // We expect that depthImage to contain depths for parts of the robot that are in the camera frame, we remove that here
      GpuMat depthImageWithoutRobot = bodyCollisionFilter.process(hostDepthImage, depthIntrinsicsCopy, cameraFrame);
      // Now we are finished with the host depth image because we have a new image without the robot, so close it
      hostDepthImage.close();

      // The controller can publish a status letting anyone listening know that the controller is aware of some amount of drift in the Z direction
      // If we have that parameter set to true, we update the heights of the height map to account for that drift
      if (heightMapParameters.getDriftOffsetFilter())
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(driftOffsetInZ, depthIntrinsicsCopy);
         }
      }

      // We expect to have knowledge of where the camera is in relation to the world so we can accurately display the height map
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      // Perform update, this actually creates the height map
      rapidHeightMapExtractor.update(depthImageWithoutRobot, depthIntrinsicsCopy, sensorToWorld, sensorToGround, groundToWorld);
      Mat croppedHeightMap = rapidHeightMapExtractor.getVisualizedHeightMap();
      // We have used the depth image without the robot, close this to avoid creating a memory leak
      depthImageWithoutRobot.close();

      // Perform a flying points filter as a post-processing step on the height map
      if (heightMapParameters.getFlyingPointsFilter())
      {
         Mat filteredCroppedHeightMapOnDevice = flyingPointsFilter.applyFilter(croppedHeightMap);
         filteredCroppedHeightMapOnDevice.copyTo(croppedHeightMap);
         filteredCroppedHeightMapOnDevice.close();
      }

      // Publish the height map to anyone who is subscribing
      OpenCVTools.compressImagePNG(croppedHeightMap, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         croppedHeightMapImageMessage,
                                                         heightMapPublisher,
                                                         cameraPose,
                                                         acquisitionTime,
                                                         rapidHeightMapExtractor.getSequenceNumber(),
                                                         croppedHeightMap.rows(),
                                                         croppedHeightMap.cols(),
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