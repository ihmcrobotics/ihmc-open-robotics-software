package us.ihmc.perception;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensors.realsense.RealSenseImageSensor;

import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * This class handles publishing the color and depth of the realsense. Its meant to be a standalone class that only touches the realsense.
 */
public class StandAloneRealsenseProcess
{
   private static final int DEPTH_IMAGE_KEY = RealSenseImageSensor.DEPTH_IMAGE_KEY;
   public static final String STAND_ALONE_REALSENSE_PROCESS = "StandAloneRealsenseProcess";
   private static final Map<Integer, ROS2Topic<? extends Packet<?>>> D455_IMAGE_TOPIC_MAP = Map.of(RealSenseImageSensor.COLOR_IMAGE_KEY,
                                                                                                   PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS,
                                                                                                   RealSenseImageSensor.DEPTH_IMAGE_KEY,
                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE);

   private final ROS2DemandGraphNode realsenseDemandNode;
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ROS2DemandGraphNode realsensePublishDemandNode;
   private final HeightMapParameters heightMapParameters;
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Object heightMapLock = new Object();

   private final RealSenseImageSensor realsenseSensor;
   private final ImageSensorPublishThread realsensePublishThread;

   private final ReferenceFrame zUpSensorFrame;
   private final ROS2TunedRigidBodyTransform realsenseTunableTransform;
   private RapidHeightMapManager heightMapManager;
   private final ReferenceFrame sensorFrame;

   public StandAloneRealsenseProcess(ROS2Node ros2Node, ROS2Helper ros2Helper, ROS2SyncedRobotModel syncedRobot, HeightMapParameters heightMapParameters)
   {
      this(ros2Node, ros2Helper, syncedRobot, heightMapParameters, null);
   }

   public StandAloneRealsenseProcess(ROS2Node ros2Node,
                                     ROS2Helper ros2Helper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     HeightMapParameters heightMapParameters,
                                     ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      this.heightMapParameters = heightMapParameters;
      ROS2DemandGraphNode heightMapDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);

      realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      realsenseDemandNode.addDependents(realsensePublishDemandNode, heightMapDemandNode);

      realsenseSensor = new RealSenseImageSensor(RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);

      sensorFrame = syncedRobot.getReferenceFrames().getSteppingCameraFrame();
      zUpSensorFrame = syncedRobot.getReferenceFrames().getSteppingCameraZUpFrame();

      realsenseTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                        PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                                        syncedRobot.getRobotModel().getSensorInformation().getSteppingCameraTransform());

      // We create a ThreadFactory here so that when profiling the thread, there is a user-friendly name to identify it with
      ThreadFactory threadFactory = new ThreadFactoryBuilder().setNameFormat(STAND_ALONE_REALSENSE_PROCESS).build();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1, threadFactory);
      scheduler.scheduleAtFixedRate(this::updateEverythingRealSense, 0, 33, TimeUnit.MILLISECONDS);

      realsenseSensor.setSensorFrameSupplier(syncedRobot.getReferenceFrames()::getSteppingCameraFrame);
      loopOnDemand(realsenseSensor.getGrabThread(), realsenseDemandNode);

      realsensePublishThread = new ImageSensorPublishThread(ros2Node, realsenseSensor, D455_IMAGE_TOPIC_MAP);
      loopOnDemand(realsensePublishThread, realsensePublishDemandNode);
   }

   private void updateEverythingRealSense()
   {
      try
      {
         realsenseSensor.waitForGrab();
         RawImage depthImage = realsenseSensor.getImage(DEPTH_IMAGE_KEY);

         // Initialize
         if (heightMapManager == null)
         {
            heightMapManager = new RapidHeightMapManager(ros2Node,
                                                         syncedRobot.getFullRobotModel(),
                                                         syncedRobot.getRobotModel().getSimpleRobotName(),
                                                         syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT),
                                                         syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT),
                                                         controllerFootstepQueueMonitor,
                                                         depthImage.getIntrinsicsCopy(),
                                                         heightMapParameters);
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

      realsenseTunableTransform.update();
   }

   public RapidHeightMapManager getHeightMapManager()
   {
      return heightMapManager;
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

   public void destroy()
   {
      realsenseDemandNode.destroy();
      realsensePublishDemandNode.destroy();
      realsenseSensor.close();
      realsensePublishThread.blockingKill();
   }

   private static void loopOnDemand(RepeatingTaskThread loopThread, ROS2DemandGraphNode demandNode)
   {
      if (!loopThread.isAlive())
         loopThread.start();

      if (demandNode.isDemanded())
         loopThread.startRepeating();

      demandNode.addDemandChangedCallback(loopThread::setRepeating);
   }
}
