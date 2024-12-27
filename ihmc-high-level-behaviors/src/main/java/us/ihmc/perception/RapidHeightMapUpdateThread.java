package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensors.ImageSensor;

public class RapidHeightMapUpdateThread extends RepeatingTaskThread
{
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2SyncedRobotModel syncedRobotModel;
   private final ReferenceFrame leftFootFrame;
   private final ReferenceFrame rightFootFrame;

   private RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame sensorFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final boolean runWithCUDA;
   private final int depthImageKey;

   public RapidHeightMapUpdateThread(ROS2PublishSubscribeAPI ros2,
                                     ROS2SyncedRobotModel syncedRobotModel,
                                     ReferenceFrame leftFootFrame,
                                     ReferenceFrame rightFootFrame,
                                     ImageSensor imageSensor,
                                     int depthImageKey,
                                     boolean runWithCUDA)
   {
      super(imageSensor.getSensorName() + RapidHeightMapUpdateThread.class.getSimpleName());

      this.ros2 = ros2;
      this.syncedRobotModel = syncedRobotModel;
      this.leftFootFrame = leftFootFrame;
      this.rightFootFrame = rightFootFrame;
      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;

      sensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();
      this.runWithCUDA = runWithCUDA;
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
            heightMapManager = new RapidHeightMapManager(ros2,
                                                         syncedRobotModel.getRobotModel(),
                                                         leftFootFrame,
                                                         rightFootFrame,
                                                         depthImage.getIntrinsicsCopy(),
                                                         runWithCUDA);
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
   }

   public HeightMapData getLatestHeightMapData()
   {
      synchronized (heightMapLock)
      {
         return heightMapManager.getLatestHeightMapData();
      }
   }

   public TerrainMapData getTerrainMapData()
   {
      synchronized (heightMapLock)
      {
         return heightMapManager.getTerrainMapData();
      }
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
