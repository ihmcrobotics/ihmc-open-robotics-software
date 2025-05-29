package us.ihmc.perception.gpuHeightMap;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.bytedeco.javacpp.BytePointer;
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
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

/**
 * This class takes care of managing the {@link RapidHeightMapExtractor}. This class can be used in a remote process, or locally as well.
 */
public class RapidHeightMapManager
{
   private final HeightMapParameters heightMapParameters;
   private final RapidHeightMapExtractor rapidHeightMapExtractor;

   private final Point3D sensorOrigin = new Point3D();
   private final FramePose3D cameraPose = new FramePose3D();
   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();
   public int sequenceNumber = 0;

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;

   private final ROS2Publisher<ImageMessage> heightMapPublisher;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   public RapidHeightMapManager(ROS2Node ros2Node,
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

      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);

      rapidHeightMapExtractor = new RapidHeightMapExtractor(heightMapParameters);

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

   public void updateAndPublishHeightMap(GpuMat latestDepthImage,
                                         Instant acquisitionTime,
                                         CameraIntrinsics depthIntrinsics,
                                         ReferenceFrame cameraFrame,
                                         ReferenceFrame cameraZUpFrame)
   {
      updateInternal(latestDepthImage, depthIntrinsics, cameraFrame, cameraZUpFrame);

      // Publish the height map to anyone who is subscribing
      Mat hostGlobalHeightMap = new Mat();

      // Don't close this mat as its being used in the extractor till that finish's
      GpuMat deviceGlobalHeightMap = rapidHeightMapExtractor.getHeightMap();
      deviceGlobalHeightMap.download(hostGlobalHeightMap);
      OpenCVTools.compressImagePNG(hostGlobalHeightMap, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         croppedHeightMapImageMessage,
                                                         heightMapPublisher,
                                                         cameraPose,
                                                         acquisitionTime,
                                                         sequenceNumber++,
                                                         hostGlobalHeightMap.rows(),
                                                         hostGlobalHeightMap.cols(),
                                                         (float) heightMapParameters.getHeightScaleFactor());

      hostGlobalHeightMap.close();
      deviceGlobalHeightMap.close();
   }

   /**
    * Update the Height Map with the latest depth image from the sensor
    */
   private void updateInternal(GpuMat latestDepthImage, CameraIntrinsics depthIntrinsicsCopy, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      // Option that gets triggered from a message sent from the user
      if (lowerHeightMapBackdropRequested.poll())
      {
         double footHeight = computeFootHeight();
         float loweredFootHeight = 1.0f;
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
      // We expect to have knowledge of where the camera is in relation to the world so we can accurately display the height map
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGround.getTranslation().setZ(sensorToWorld.getTranslation().getZ());

      sensorOrigin.set(sensorToWorld.getTranslation());
      cameraPose.setToZero(cameraZUpFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      // The controller can publish a status letting anyone listening know that the controller is aware of some amount of drift in the Z direction
      // If we have that parameter set to true, we update the heights of the height map to account for that drift
      if (heightMapParameters.getDriftOffsetFilter())
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(groundToWorld, driftOffsetInZ, depthIntrinsicsCopy, computeFootHeight());
         }
      }

      // Perform update, this actually creates the height map
      rapidHeightMapExtractor.update(latestDepthImage, depthIntrinsicsCopy, sensorToWorld, sensorToGround, groundToWorld, sensorOrigin, computeFootHeight());
   }

   public HeightMapData getLatestHeightMapData()
   {
      HeightMapData latestHeightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                                            sensorOrigin.getX(),
                                                            sensorOrigin.getY());
      Mat heightMap = new Mat();
      GpuMat deviceTerrainHeightMap = rapidHeightMapExtractor.getTerrainHeightMap();
      deviceTerrainHeightMap.download(heightMap);
      PerceptionMessageTools.convertToHeightMapData(heightMap,
                                                    latestHeightMapData,
                                                    sensorOrigin,
                                                    (float) heightMapParameters.getTerrainWidthInMeters(),
                                                    (float) heightMapParameters.getCellSizeInMeters(),
                                                    heightMapParameters);
      heightMap.close();
      deviceTerrainHeightMap.close();

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
      compressedCroppedHeightMapPointer.close();
      heightMapPublisher.remove();
      rapidHeightMapExtractor.destroy();
   }
}