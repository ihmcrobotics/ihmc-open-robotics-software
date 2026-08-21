package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.ImageMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ActiveMappingParameterToolBox;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.log.LogTools;
import us.ihmc.perception.filters.DepthImageBodyCollisionFilter;
import us.ihmc.perception.filters.DepthImageFlyingPointsFilter;
import us.ihmc.perception.gpuMapping.GpuMappingManager;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.CameraIntrinsics;

import java.util.concurrent.BlockingQueue;

public class GpuMappingThread extends RepeatingTaskThread
{
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;
   private final DepthImageFlyingPointsFilter flyingPointsFilter;
   private final GpuMappingManager gpuMappingManager;
   private final Object terrainMapLock = new Object();

   private final HeightMapParameters heightMapParameters;
   private final ROS2Publisher<ImageMessage> filteredDepthPublisher;
   private final BlockingQueue<RawImage> rawImageCollection;
   private final ImageMessage filteredDepthImageMessage = new ImageMessage();

   private final ROS2DemandGraphNode publishChunkMap;
   private final ROS2DemandGraphNode publishHeightMap;
   private final ROS2DemandGraphNode publishHeightMapToController;
   private final ROS2DemandGraphNode publishTerrainMap;

   public GpuMappingThread(ROS2Node ros2Node,
                           ROS2SyncedRobotModel syncedRobotModel,
                           RobotCollisionModel robotCollisionModel,
                           BlockingQueue<RawImage> rawImageCollection,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ActiveMappingParameterToolBox activeMappingParameterToolBox)
   {
      super(GpuMappingThread.class.getSimpleName());
      this.rawImageCollection = rawImageCollection;
      this.heightMapParameters = activeMappingParameterToolBox.getHeightMapParameters();

      publishChunkMap = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_CHUNK_MAP);
      publishHeightMap = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);
      publishHeightMapToController = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP_FOR_CONTROLLER);
      publishTerrainMap = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_TERRAIN_MAP);

      // At the highest level pass in the reference frames for the specific robot
      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.RIGHT);
      // TODO we don't have a great way to setup the height map if we are using more then one sensor
      // This will make the height map not appear correct cause the center is wrong
      ReferenceFrame heightMapCenterFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();

      filteredDepthPublisher = ros2Node.createPublisher(PerceptionAPI.STEPPING_REALSENSE_DEPTH_FILTERED);

      bodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, syncedRobotModel.getFullRobotModel().getRootBody());
      flyingPointsFilter = new DepthImageFlyingPointsFilter(activeMappingParameterToolBox.getDepthImageFilteringParameters());
      gpuMappingManager = new GpuMappingManager(syncedRobotModel.getRobotModel().getSimpleRobotName(),
                                                ros2Node,
                                                leftFootFrame,
                                                rightFootFrame,
                                                heightMapCenterFrame,
                                                controllerFootstepQueueMonitor,
                                                heightMapParameters,
                                                activeMappingParameterToolBox.getTerrainMapParameters());
   }

   @Override
   protected void runTask()
   {
      try
      {
         RawImage depthImage = rawImageCollection.take();

         // We can get the transform to world from the image and use that to get the desired camera frames
         RigidBodyTransformReadOnly transformToWorld = depthImage.getTransformToWorld();
         ReferenceFrame cameraFrameInWorld = new FixedReferenceFrame("FrameInWorld", ReferenceFrame.getWorldFrame(), transformToWorld);
         ZUpFrame cameraZUpFrameInWorld = new ZUpFrame(cameraFrameInWorld, "ZUpFrameInWorld");
         // Need to update this due to how its implemented, other the transform to world will be all zeros
         cameraZUpFrameInWorld.update();

         // Get everything we need from the image
         GpuMat latestDepthImage = depthImage.getGpuImageMat();
         CameraIntrinsics depthIntrinsicsCopy = depthImage.getIntrinsicsCopy();
         GpuMat filteredDepthImage = new GpuMat(latestDepthImage.size(), latestDepthImage.type());

         // Process body collisions
         bodyCollisionFilter.process(latestDepthImage, filteredDepthImage, depthIntrinsicsCopy, cameraFrameInWorld);

         if (heightMapParameters.getFlyingPointsFilter())
         {
            GpuMat depthImageNoFlyingPoints = new GpuMat(filteredDepthImage.size(), filteredDepthImage.type());
            flyingPointsFilter.applyFilter(filteredDepthImage, depthImageNoFlyingPoints, depthIntrinsicsCopy);
            depthImageNoFlyingPoints.copyTo(filteredDepthImage);

            CompressionType compressionType;
            BytePointer bytePointer;

            compressionType = CompressionType.PNG;
            Mat cpuDepthImage = new Mat();
            bytePointer = new BytePointer();
            depthImageNoFlyingPoints.download(cpuDepthImage);
            OpenCVTools.compressImagePNG(cpuDepthImage, bytePointer);
            cpuDepthImage.close();

            ImageMessage imageMessage = filteredDepthImageMessage;
            imageMessage.getData().clear();
            PerceptionMessageTools.packImageMessage(depthImage, bytePointer, compressionType, imageMessage);
            filteredDepthPublisher.publish(imageMessage);

            bytePointer.close();

            depthImageNoFlyingPoints.close();
         }

         // Update height map
         synchronized (terrainMapLock)
         {
            gpuMappingManager.update(filteredDepthImage, depthIntrinsicsCopy, cameraFrameInWorld, cameraZUpFrameInWorld);
         }

         // Publish the updated maps if demanded
         if (publishHeightMap.isDemanded())
            gpuMappingManager.publishHeightMap();
         if (publishHeightMapToController.isDemanded())
            gpuMappingManager.publishHeightMapForController();
         if (publishTerrainMap.isDemanded())
            gpuMappingManager.publishTerrainMap();

         // Chunked map is only published if it's enabled by the height map parameters
         if (publishChunkMap.isDemanded())
         {
            gpuMappingManager.updateChunkedMap();
            gpuMappingManager.publishChunkedMap();
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
      synchronized (terrainMapLock)
      {
         return gpuMappingManager.getLatestHeightMapData();
      }
   }

   public TerrainMapData getlatestTerrainMapData()
   {
      synchronized (terrainMapLock)
      {
         return gpuMappingManager.getLatestTerrainMapData();
      }
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      publishChunkMap.destroy();
      publishHeightMap.destroy();
      publishHeightMapToController.destroy();
      publishTerrainMap.destroy();

      bodyCollisionFilter.close();
      flyingPointsFilter.destroy();
      gpuMappingManager.destroy();
   }
}
