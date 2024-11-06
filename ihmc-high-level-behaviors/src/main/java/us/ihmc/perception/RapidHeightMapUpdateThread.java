package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensors.ImageSensor;

public class RapidHeightMapUpdateThread extends RepeatingTaskThread
{
   private final ROS2PublishSubscribeAPI ros2;
   private final DRCRobotModel robotModel;
   private final ReferenceFrame leftFootFrame;
   private final ReferenceFrame rightFootFrame;
   private final OpenCLManager openCLManager;

   private RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final MutableReferenceFrame sensorFrame;
   private final MutableReferenceFrame zUpSensorFrame;
   private final int depthImageKey;

   public RapidHeightMapUpdateThread(ROS2PublishSubscribeAPI ros2,
                                     DRCRobotModel robotModel,
                                     ReferenceFrame leftFootFrame,
                                     ReferenceFrame rightFootFrame,
                                     OpenCLManager openCLManager,
                                     ImageSensor imageSensor,
                                     int depthImageKey)
   {
      super(imageSensor.getSensorName() + RapidHeightMapUpdateThread.class.getSimpleName());

      this.ros2 = ros2;
      this.robotModel = robotModel;
      this.leftFootFrame = leftFootFrame;
      this.rightFootFrame = rightFootFrame;
      this.openCLManager = openCLManager;
      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;

      sensorFrame = new MutableReferenceFrame();
      zUpSensorFrame = new MutableReferenceFrame(sensorFrame.getReferenceFrame());
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
            heightMapManager = new RapidHeightMapManager(openCLManager, robotModel, leftFootFrame, rightFootFrame, depthImage.getIntrinsicsCopy(), ros2);

         // Update sensor frames
         sensorFrame.update(transformToWorld -> transformToWorld.set(depthImage.getPose()));
         zUpSensorFrame.update(transformToWorld ->
         {
            transformToWorld.setRotationToZero();
            transformToWorld.appendYawRotation(sensorFrame.getTransformToParent().getRotation().getYaw());
         });

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.update(depthImage.getCpuImageMat(),
                                    depthImage.getAcquisitionTime(),
                                    sensorFrame.getReferenceFrame(),
                                    zUpSensorFrame.getReferenceFrame(),
                                    ros2);
         }

         depthImage.release();
      } catch (InterruptedException ignored) {}
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

      heightMapManager.destroy();
   }
}
