package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

public class RapidHeightMapUpdateThread extends RepeatingTaskThread
{
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobotModel;
   private final ReferenceFrame leftFootFrame;
   private final ReferenceFrame rightFootFrame;

   private RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ImageSensor imageSensor;
   private final ReferenceFrame sensorFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final HeightMapParameters heightMapParameters;
   private final int depthImageKey;
   private final RobotCollisionModel robotCollisionModel;

   public RapidHeightMapUpdateThread(ROS2Node ros2Node,
                                     ROS2SyncedRobotModel syncedRobotModel,
                                     RobotCollisionModel robotCollisionModel,
                                     ReferenceFrame leftFootFrame,
                                     ReferenceFrame rightFootFrame,
                                     ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                     ImageSensor imageSensor,
                                     int depthImageKey,
                                     HeightMapParameters heightMapParameters)
   {
      super(imageSensor.getSensorName() + RapidHeightMapUpdateThread.class.getSimpleName());

      this.ros2Node = ros2Node;
      this.syncedRobotModel = syncedRobotModel;
      this.robotCollisionModel = robotCollisionModel;
      this.leftFootFrame = leftFootFrame;
      this.rightFootFrame = rightFootFrame;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;
      this.heightMapParameters = heightMapParameters;

      sensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();
   }

   @Override
   protected void runTask()
   {
      try
      {
         imageSensor.waitForGrab();
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Initialize
         if (heightMapManager == null)
         {
            heightMapManager = new RapidHeightMapManager(ros2Node,
                                                         robotCollisionModel,
                                                         syncedRobotModel.getFullRobotModel(),
                                                         syncedRobotModel.getRobotModel().getSimpleRobotName(),
                                                         leftFootFrame,
                                                         rightFootFrame,
                                                         controllerFootstepQueueMonitor,
                                                         depthImage.getIntrinsicsCopy(),
                                                         heightMapParameters,
                                                         sensorFrame);
         }

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.update(depthImage.getCpuImageMat(), depthImage.getAcquisitionTime(), sensorFrame, zUpSensorFrame);
         }

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

   public TerrainMapData getLatestTerrainMapData()
   {
      synchronized (heightMapLock)
      {
         return heightMapManager.getTerrainMapData();
      }
   }

   public RapidHeightMapManager getHeightMapManager()
   {
      return heightMapManager;
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      if (heightMapManager != null)
         heightMapManager.destroy();
   }
}
