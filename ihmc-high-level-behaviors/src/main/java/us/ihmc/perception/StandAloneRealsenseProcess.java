package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensors.realsense.RealSenseImageSensor;

import java.util.Map;

/**
 * This class handles publishing the color and depth of the realsense. Its meant to be a standalone class that only touches the realsense.
 */
public class StandAloneRealsenseProcess
{
   private static final Map<Integer, ROS2Topic<? extends Packet<?>>> D455_IMAGE_TOPIC_MAP = Map.of(RealSenseImageSensor.COLOR_IMAGE_KEY,
                                                                                                   PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS,
                                                                                                   RealSenseImageSensor.DEPTH_IMAGE_KEY,
                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE);

   private final ROS2DemandGraphNode realsenseDemandNode;
   private final ROS2DemandGraphNode realsensePublishDemandNode;
   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;

   private final RealSenseImageSensor d455Sensor;
   private final ImageSensorPublishThread d455PublishThread;

   private final ROS2DemandGraphNode heightMapDemandNode;
   private RapidHeightMapUpdateThread heightMapUpdateThread;

   public StandAloneRealsenseProcess(ROS2Node ros2Node, ROS2Helper ros2Helper, ROS2SyncedRobotModel syncedRobot)
   {
      this(ros2Node, ros2Helper, syncedRobot, null);
   }

   public StandAloneRealsenseProcess(ROS2Node ros2Node,
                                     ROS2Helper ros2Helper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {
      this.ros2Helper = ros2Helper;
      this.syncedRobot = syncedRobot;

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      heightMapDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);

      realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      realsenseDemandNode.addDependents(realsensePublishDemandNode, heightMapDemandNode);

      d455Sensor = new RealSenseImageSensor(RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);

      ROS2TunedRigidBodyTransform realsenseTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                                                    PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                                                                    syncedRobot.getRobotModel()
                                                                                                               .getSensorInformation()
                                                                                                               .getSteppingCameraTransform());

      RepeatingTaskThread realsenseUpdateThread = new RepeatingTaskThread("SyncedRobotUpdate", realsenseTunableTransform::update);
      realsenseUpdateThread.setFrequencyLimit(30.0);
      realsenseUpdateThread.startRepeating();

      d455Sensor.setSensorFrameSupplier(syncedRobot.getReferenceFrames()::getSteppingCameraFrame);
      loopOnDemand(d455Sensor.getGrabThread(), realsenseDemandNode);

      d455PublishThread = new ImageSensorPublishThread(ros2Node, d455Sensor, D455_IMAGE_TOPIC_MAP);
      loopOnDemand(d455PublishThread, realsensePublishDemandNode);

      initializeHeightMap(controllerFootstepQueueMonitor);
   }

   private void initializeHeightMap(ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {
      boolean runWithCUDA = true;
      heightMapUpdateThread = new RapidHeightMapUpdateThread(ros2Helper.getROS2Node(),
                                                             syncedRobot,
                                                             syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT),
                                                             syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT),
                                                             controllerFootstepQueueMonitor,
                                                             d455Sensor,
                                                             RealSenseImageSensor.DEPTH_IMAGE_KEY,
                                                             runWithCUDA);
      loopOnDemand(heightMapUpdateThread, heightMapDemandNode);
   }

   public RapidHeightMapManager getHeightMapManager()
   {
      return heightMapUpdateThread.getHeightMapManager();
   }

   public HeightMapData getLatestHeightMapData()
   {
      return heightMapUpdateThread.getLatestHeightMapData();
   }

   public TerrainMapData getLatestTerrainMapData()
   {
      return heightMapUpdateThread.getLatestTerrainMapData();
   }

   public void destroy()
   {
      realsenseDemandNode.destroy();
      realsensePublishDemandNode.destroy();
      d455Sensor.close();
      d455PublishThread.blockingKill();
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
