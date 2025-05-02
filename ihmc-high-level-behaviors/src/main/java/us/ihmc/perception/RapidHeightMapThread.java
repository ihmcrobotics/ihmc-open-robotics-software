package us.ihmc.perception;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.ImageSensor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

public class RapidHeightMapThread
{
   private final RapidHeightMapManager heightMapManager;
   private final Object heightMapLock = new Object();

   private final ImageSensor imageSensor;
   private final ReferenceFrame sensorFrame;
   private final ReferenceFrame zUpSensorFrame;
   private final int depthImageKey;
   private ScheduledExecutorService scheduler;

   public RapidHeightMapThread(ROS2Node ros2Node,
                               ROS2SyncedRobotModel syncedRobotModel,
                               RobotCollisionModel robotCollisionModel,
                               ImageSensor imageSensor,
                               int depthImageKey,
                               ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                               HeightMapParameters heightMapParameters)
   {
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

   public void start()
   {
      // We create a ThreadFactory here so that when profiling the thread, there is a user-friendly name to identify it with
      ThreadFactory threadFactory = new ThreadFactoryBuilder().setNameFormat("RemoteHeightMapUpdateThread").build();
      scheduler = Executors.newScheduledThreadPool(1, threadFactory);
      scheduler.scheduleWithFixedDelay(this::update, 100, 10, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      try
      {
         imageSensor.waitForGrab();
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Update height map
         synchronized (heightMapLock)
         {
            heightMapManager.updateAndPublishHeightMap(depthImage, sensorFrame, zUpSensorFrame);
         }

         depthImage.release();
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

   public void destroy()
   {
      scheduler.shutdown();
      heightMapManager.destroy();
   }
}
