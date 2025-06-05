package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

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
   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;

   private final ROS2Publisher<HeightMapMessage> heightMapMessagePublisher;
   private final BytePointer compressedHeightMapPointer = new BytePointer();
   private final HeightMapData latestHeightMapData;
   private final HeightMapData latestTerrainHeightMapData;
   private final Point3D gridCellLocation = new Point3D();
   private long sequenceId = 0;

   public RapidHeightMapManager(ROS2Node ros2Node,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
      latestHeightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                              (float) heightMapParameters.getGlobalWidthInMeters(),
                                              0.0,
                                              0.0);
      latestTerrainHeightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                                     (float) heightMapParameters.getTerrainWidthInMeters(),
                                                     0.0,
                                                     0.0);

      footSoleFrames.add(leftFootSoleFrame);
      footSoleFrames.add(rightFootSoleFrame);

      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);

      rapidHeightMapExtractor = new RapidHeightMapExtractor(heightMapParameters);

      // We use a notification to only call resetting the height map in one place
      heightMapMessagePublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_MESSAGE);
      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      ros2Node.createSubscription2(PerceptionAPI.LOWER_HEIGHT_MAP_BACKDROP, message -> lowerHeightMapBackdropRequested.set());
   }

   public void updateAndPublishHeightMap(GpuMat latestDepthImage, CameraIntrinsics depthIntrinsics, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      updateInternal(latestDepthImage, depthIntrinsics, cameraFrame, cameraZUpFrame);

      // Publish the height map to anyone who is subscribing
      Mat hostGlobalHeightMap = new Mat();

      // Don't close this mat as its being used in the extractor till that finish's
      GpuMat deviceGlobalHeightMap = rapidHeightMapExtractor.getHeightMap();
      deviceGlobalHeightMap.download(hostGlobalHeightMap);

      publishHeightMap(hostGlobalHeightMap);

      hostGlobalHeightMap.close();
      deviceGlobalHeightMap.close();
   }

   private void publishHeightMap(Mat hostGlobalHeightMap)
   {
      // The center of this map should be centered in the world grid
      // The sensor origin isn't always at the center of a grid point, in fact it's often not in the center
      int currentCellX = (int) Math.round(sensorOrigin.getX32() / heightMapParameters.getCellSizeInMeters());
      int currentCellY = (int) Math.round(sensorOrigin.getY32() / heightMapParameters.getCellSizeInMeters());
      gridCellLocation.set(currentCellX * 0.02, currentCellY * 0.02, 0.0);
      FramePose3D cameraPose = new FramePose3D();
      cameraPose.getTranslation().set(gridCellLocation);

      HeightMapMessageTools.convertToHeightMapData(hostGlobalHeightMap,
                                                   latestHeightMapData,
                                                   gridCellLocation,
                                                   (float) heightMapParameters.getGlobalWidthInMeters(),
                                                   (float) heightMapParameters.getCellSizeInMeters(),
                                                   heightMapParameters);

      HeightMapMessage heightMapMessage = new HeightMapMessage();
      HeightMapMessageTools.toMessage(latestHeightMapData, heightMapMessage);
      sequenceId++;
      heightMapMessage.setSequenceId(sequenceId);
      heightMapMessagePublisher.publish(heightMapMessage);
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
      GpuMat terrainCroppedHeightMap = rapidHeightMapExtractor.getTerrainCroppedHeightMap();
      Mat terrainHeightMap = new Mat();
      terrainCroppedHeightMap.download(terrainHeightMap);

      HeightMapMessageTools.convertToHeightMapData(terrainHeightMap,
                                                   latestTerrainHeightMapData,
                                                   gridCellLocation,
                                                   (float) heightMapParameters.getTerrainWidthInMeters(),
                                                   (float) heightMapParameters.getCellSizeInMeters(),
                                                   heightMapParameters);
      return latestTerrainHeightMapData;
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
      compressedHeightMapPointer.close();
      heightMapMessagePublisher.remove();
      rapidHeightMapExtractor.destroy();
   }
}