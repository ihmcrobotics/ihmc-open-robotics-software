package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

public class RapidHeightMapUpdateThread extends RepeatingTaskThread
{

   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame sensorFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final int depthImageKey;

   public RapidHeightMapUpdateThread(ROS2Node ros2Node,
                                     ROS2SyncedRobotModel syncedRobotModel,
                                     RobotCollisionModel robotCollisionModel,
                                     ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                     ImageSensor imageSensor,
                                     int depthImageKey,
                                     HeightMapParameters heightMapParameters)
   {
      super(imageSensor.getSensorName() + RapidHeightMapUpdateThread.class.getSimpleName());

      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;

      sensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobotModel.getReferenceFrames().getSteppingCameraZUpFrame();

      ReferenceFrame leftFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT);

      heightMapManager = new RapidHeightMapManager(ros2Node,
                                                   robotCollisionModel,
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
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.update(depthImage, sensorFrame, zUpSensorFrame);
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

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      heightMapManager.destroy();
   }
}
