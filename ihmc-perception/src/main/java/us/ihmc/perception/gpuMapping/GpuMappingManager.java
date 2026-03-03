package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessageForController;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.gpuMapping.worldModel.ChunkedMapManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.util.ArrayList;
import java.util.List;

/**
 * This class takes care of managing the {@link HeightMapExtractor}. This class can be used in a remote process, or locally as well.
 */
public class GpuMappingManager
{
   private final ReferenceFrame heightMapCenter;
   private final HeightMapParameters heightMapParameters;
   private final HeightMapDriftOffset heightMapDriftOffset;
   private final HeightMapExtractor heightMapExtractor;
   private final TerrainMapExtractor terrainMapExtractor;
   private final ChunkedMapManager chunkedMapManager;

   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final ROS2Publisher<HeightMapMessage> heightMapMessagePublisher;
   private final ROS2Publisher<HeightMapMessageForController> controllerHeightMapMessagePublisher;
   private final ROS2Publisher<TerrainMapMessage> terrainMapMessagePublisher;
   private final BytePointer compressedHeightMapPointer = new BytePointer();
   private final Point3D heightMapCenterPoint = new Point3D();

   // These fields are created globally cause it takes compute time to create it in the update loop
   private final HeightMapMessage heightMapMessage;
   private final HeightMapMessageForController heightMapMessageForController;
   private long heightMapSequenceId = 0;
   private long heightMapForControllerSequenceId = 0;
   private final TerrainMapMessage terrainMapMessage;
   private long terrainMapSequenceId = 0;

   public GpuMappingManager(String robotName,
                            ROS2Node ros2Node,
                            ReferenceFrame leftFootSoleFrame,
                            ReferenceFrame rightFootSoleFrame,
                            ReferenceFrame heightMapCenter,
                            ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                            HeightMapParameters heightMapParameters,
                            TerrainMapParameters terrainMapParameters)
   {
      this.heightMapCenter = heightMapCenter;
      this.heightMapParameters = heightMapParameters;

      footSoleFrames.add(leftFootSoleFrame);
      footSoleFrames.add(rightFootSoleFrame);

      heightMapDriftOffset = new HeightMapDriftOffset(controllerFootstepQueueMonitor);
      heightMapExtractor = new HeightMapExtractor(heightMapParameters);
      terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, terrainMapParameters);
      chunkedMapManager = new ChunkedMapManager(ros2Node, heightMapParameters);

      // Again we do this to optimize the speed of the rapid height map
      heightMapMessage = new HeightMapMessage();
      heightMapMessageForController = new HeightMapMessageForController();
      terrainMapMessage = new TerrainMapMessage();

      // We use a notification to only call resetting the height map in one place
      heightMapMessagePublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_MESSAGE);
      terrainMapMessagePublisher = ros2Node.createPublisher(PerceptionAPI.TERRAIN_MAP_MESSAGE);
      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      ros2Node.createSubscription2(PerceptionAPI.LOWER_HEIGHT_MAP_BACKDROP, message -> lowerHeightMapBackdropRequested.set());

      controllerHeightMapMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(HeightMapMessageForController.class, robotName));
   }

   /**
    * Update the Height Map with the latest depth image from the sensor
    */
   public void update(GpuMat latestDepthImage, CameraIntrinsics depthIntrinsics, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame, ReferenceFrame midFeetZUpFrame)
   {
      // Option that gets triggered from a message sent from the user
      if (lowerHeightMapBackdropRequested.poll())
      {
         double footHeight = computeFootHeight();
         float loweredFootHeight = 1.0f;
         heightMapExtractor.reset(footHeight, loweredFootHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            heightMapDriftOffset.reset();
         }
      }

      // Option that gets triggered from a message sent from the user
      if (resetHeightMapRequested.poll())
      {
         double footHeight = computeFootHeight();
         heightMapExtractor.reset(footHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            heightMapDriftOffset.reset();
         }
      }

      // Get robot yaw
      FrameQuaternion robotOrientation = new FrameQuaternion(midFeetZUpFrame);
      robotOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double robotYaw = robotOrientation.getYaw();

      // Update the sensor origin here with the latest reference frame
      // We are deep coping the frames here to avoid a data race condition, still possible but very small chance
      RigidBodyTransform heightMapFrameToWorldFrame = new RigidBodyTransform(heightMapCenter.getTransformToWorldFrame());
      Point3D heightMapCenterOrigin = new Point3D(heightMapFrameToWorldFrame.getTranslation());

      // -------- Update the Height Map with the latest depth image from the sensor --------------
      // We expect to have knowledge of where the camera is in relation to the world so we can accurately display the height map
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGround.getTranslation().setZ(sensorToWorld.getTranslation().getZ());

      // The controller can publish a status letting anyone listening know that the controller is aware of some amount of drift in the Z direction
      // If we have that parameter set to true, we update the heights of the height map to account for that drift
      float driftOffsetInZ = 0;
      if (heightMapParameters.getDriftOffsetFilter())
      {
         driftOffsetInZ = heightMapDriftOffset.getUpdateDriftOffset();
      }

      // Perform update, this actually creates the height map
      heightMapExtractor.update(latestDepthImage,
                                depthIntrinsics,
                                sensorToWorld,
                                sensorToGround,
                                groundToWorld,
                                driftOffsetInZ,
                                heightMapCenterOrigin,
                                computeFootHeight());

      terrainMapExtractor.update(heightMapExtractor.getHeightMap(), heightMapCenterPoint, robotYaw);

      // The center of this map should be centered in the world grid
      // The sensor origin isn't always at the center of a grid point, in fact it's often not in the center
      double currentCellX = (int) Math.round(heightMapCenterOrigin.getX32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      double currentCellY = (int) Math.round(heightMapCenterOrigin.getY32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      heightMapCenterPoint.set(currentCellX, currentCellY, 0.0);
   }

   public void publishHeightMap()
   {
      HeightMapMessageTools.toMessage(heightMapExtractor.getHeightMapData(), heightMapMessage);

      //      heightMapLogger.logHeightMap(globalHeightMap, heightMapCenterPoint);

      heightMapMessage.setSequenceId(heightMapSequenceId++);
      heightMapMessagePublisher.publish(heightMapMessage);
   }

   public void publishHeightMapForController()
   {
      HeightMapMessageTools.toMessageForController(heightMapExtractor.getHeightMapData(), heightMapMessageForController);
      heightMapMessageForController.setSequenceId(heightMapForControllerSequenceId++);
      controllerHeightMapMessagePublisher.publish(heightMapMessageForController);

   }

    public void publishTerrainMap()
    {
        TerrainMapMessageTools.toMessage(terrainMapExtractor.getTerrainMapData(), terrainMapMessage);
        terrainMapMessage.setSequenceId(terrainMapSequenceId++);

        terrainMapMessagePublisher.publish(terrainMapMessage);
    }

    public void updateChunkedMap()
    {
       // Publish the height map to anyone who is subscribing
       Mat hostGlobalHeightMap = new Mat();
       // Don't destroy this mat as its being used in the extractor till that finish's
       GpuMat deviceGlobalHeightMap = heightMapExtractor.getHeightMap();
       deviceGlobalHeightMap.download(hostGlobalHeightMap);

       chunkedMapManager.update(hostGlobalHeightMap, heightMapCenterPoint);

       hostGlobalHeightMap.close();
    }

   public void publishChunkedMap()
   {
      chunkedMapManager.publishChunkedMap();
   }

   public HeightMapData getLatestHeightMapData()
   {
      return heightMapExtractor.getHeightMapData();
   }

   public TerrainMapData getLatestTerrainMapData()
   {
       return terrainMapExtractor.getTerrainMapData();
   }

   private double computeFootHeight()
   {
      double thicknessOfTheFoot = 0.02;
      double height = Double.POSITIVE_INFINITY;

      // We are deep coping the frames here to avoid a data race condition, still possible but very small chance
      RigidBodyTransform[] feetTransforms = new RigidBodyTransform[footSoleFrames.size()];
      for (int i = 0; i < footSoleFrames.size(); i++)
      {
         feetTransforms[i] = new RigidBodyTransform(footSoleFrames.get(i).getTransformToWorldFrame());
      }

      for (RigidBodyTransform transform : feetTransforms)
      {
         height = Math.min(transform.getTranslationZ(), height);
      }

      if (Double.isInfinite(height))
         height = 0.0;

      height -= thicknessOfTheFoot;

      return height;
   }

   public void destroy()
   {
      compressedHeightMapPointer.close();
      controllerHeightMapMessagePublisher.remove();
      heightMapMessagePublisher.remove();
      heightMapExtractor.destroy();
      terrainMapExtractor.destroy();
      chunkedMapManager.destroy();
   }
}