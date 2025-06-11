package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.filters.DepthImageBodyCollisionFilter;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.perception.filters.DepthImageFlyingPointsFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

import java.util.concurrent.BlockingQueue;

public class RapidHeightMapThread extends RepeatingTaskThread
{
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;
   private final DepthImageFlyingPointsFilter flyingPointsFilter;
   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final HeightMapParameters heightMapParameters;
   private final CUDACompressionTools cudaCompressionTools = new CUDACompressionTools();
   private final ROS2Publisher<ImageMessage> filteredDepthPublisher;
   private final BlockingQueue<RawImage> rawImageCollection;

   public RapidHeightMapThread(ROS2Node ros2Node,
                               ROS2SyncedRobotModel syncedRobotModel,
                               RobotCollisionModel robotCollisionModel,
                               BlockingQueue<RawImage> rawImageCollection,
                               ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                               HeightMapParameters heightMapParameters,
                               DepthImageFilteringParameters depthImageFilteringParameters)
   {
      super(RapidHeightMapThread.class.getSimpleName());
      this.rawImageCollection = rawImageCollection;
      this.heightMapParameters = heightMapParameters;

      // At the highest level pass in the reference frames for the specific robot
      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame heightMapCenterFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();

      filteredDepthPublisher = ros2Node.createPublisher(PerceptionAPI.REALSENSE_DEPTH_FILTERED_IMAGE);

      bodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, syncedRobotModel.getFullRobotModel().getRootBody());
      flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
      heightMapManager = new RapidHeightMapManager(ros2Node,
                                                   leftFootFrame,
                                                   rightFootFrame,
                                                   heightMapCenterFrame,
                                                   controllerFootstepQueueMonitor,
                                                   heightMapParameters);
   }

   @Override
   protected void runTask()
   {
      try
      {
         RawImage depthImage = rawImageCollection.take();

         // We can get the transform to world from the image and use that to get the desired camera frames
         RigidBodyTransformReadOnly transformToWorld = depthImage.getTransformToWorld();
         ReferenceFrame cameraFrameInWorld = new FixedReferenceFrame("RealsenseFrameInWorld", ReferenceFrame.getWorldFrame(), transformToWorld);
         ZUpFrame cameraZUpFrameInWorld = new ZUpFrame(cameraFrameInWorld, "RealsenseZUpFrameInWorld");
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

            BytePointer bytePointer = cudaCompressionTools.compressDepth(depthImageNoFlyingPoints);

            ImageMessage imageMessage = new ImageMessage();
            PerceptionMessageTools.packImageMessage(depthImage, bytePointer, CompressionType.ZSTD_NVJPEG_HYBRID, imageMessage);
            filteredDepthPublisher.publish(imageMessage);

            depthImageNoFlyingPoints.close();
         }

         // Update height map
         synchronized (heightMapLock)
         {
            LogTools.info("Start...");
            heightMapManager.updateAndPublishHeightMap(filteredDepthImage, depthIntrinsicsCopy, cameraFrameInWorld, cameraZUpFrameInWorld);
            LogTools.info("END");
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

      cudaCompressionTools.destroy();
      bodyCollisionFilter.close();
      flyingPointsFilter.destroy();
      heightMapManager.destroy();
   }
}
