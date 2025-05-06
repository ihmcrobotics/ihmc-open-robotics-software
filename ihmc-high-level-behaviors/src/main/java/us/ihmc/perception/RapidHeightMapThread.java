package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.filters.DepthImageBodyCollisionFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;

public class RapidHeightMapThread extends RepeatingTaskThread
{
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;
   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame cameraFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final int depthImageKey;

   public RapidHeightMapThread(ROS2Node ros2Node,
                               ROS2SyncedRobotModel syncedRobotModel,
                               RobotCollisionModel robotCollisionModel,
                               ImageSensor imageSensor,
                               int depthImageKey,
                               ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                               HeightMapParameters heightMapParameters)
   {
      super(imageSensor.getSensorName() + RapidHeightMapThread.class.getSimpleName());

      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;

      cameraFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();

      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);

      bodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, syncedRobotModel.getFullRobotModel().getRootBody());
      heightMapManager = new RapidHeightMapManager(ros2Node,
                                                   syncedRobotModel.getFullRobotModel(),
                                                   syncedRobotModel.getRobotModel().getSimpleRobotName(),
                                                   leftFootFrame,
                                                   rightFootFrame,
                                                   controllerFootstepQueueMonitor,
                                                   heightMapParameters);
   }

   @Override
   protected void runTask()
   {
      try
      {
         imageSensor.waitForGrab();
         RawImage depthImage = imageSensor.getImage(depthImageKey).get();

         // Get everything we need from the image
         RawImage depthImageCopy = depthImage.get();
         GpuMat latestDepthImage = depthImageCopy.getGpuImageMat();
         Instant acquisitionTime = depthImageCopy.getAcquisitionTime();
         CameraIntrinsics depthIntrinsicsCopy = depthImageCopy.getIntrinsicsCopy();

         // Process body collisions
         GpuMat depthImageWithoutBodyCollisions = new GpuMat(latestDepthImage.size(), latestDepthImage.type());
         bodyCollisionFilter.process(latestDepthImage, depthImageWithoutBodyCollisions, depthIntrinsicsCopy, cameraFrame);

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.updateAndPublishHeightMap(depthImageWithoutBodyCollisions, acquisitionTime, depthIntrinsicsCopy, cameraFrame, zUpSensorFrame);
         }

         depthImageWithoutBodyCollisions.close();
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

      heightMapManager.destroy();
   }
}
