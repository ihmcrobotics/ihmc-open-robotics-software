package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensors.RealsenseImageSensor;

import java.util.Map;

/**
 * This class handles publishing the color and depth of the realsense. Its meant to be a standalone class that only touches the realsense.
 */
public class StandAloneRealsenseProcess
{
   private static final Map<Integer, ROS2Topic<? extends Packet<?>>> D455_IMAGE_TOPIC_MAP = Map.of(RealsenseImageSensor.COLOR_IMAGE_KEY,
                                                                                                   PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS,
                                                                                                   RealsenseImageSensor.DEPTH_IMAGE_KEY,
                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE);

   private final ROS2DemandGraphNode realsenseDemandNode;
   private final ROS2DemandGraphNode realsensePublishDemandNode;
   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;

   private final RealsenseImageSensor d455Sensor;
   private ImageSensorPublishThread d455PublishThread;

   private final ROS2DemandGraphNode heightMapDemandNode;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private RapidHeightMapUpdateThread heightMapUpdateThread;

   public StandAloneRealsenseProcess(ROS2Node ros2Node, ROS2Helper ros2Helper, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2Helper = ros2Helper;
      this.syncedRobot = syncedRobot;

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      heightMapDemandNode = new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_HEIGHT_MAP);

      realsenseDemandNode = new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_REALSENSE);
      realsenseDemandNode.addDependents(realsensePublishDemandNode, heightMapDemandNode);

      d455Sensor = new RealsenseImageSensor(RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);

      if (syncedRobot != null)
      {
         d455Sensor.setSensorFrameSupplier(syncedRobot.getReferenceFrames()::getSteppingCameraFrame);
         loopOnDemand(d455Sensor.getGrabThread(), realsenseDemandNode);

         d455PublishThread = new ImageSensorPublishThread(ros2Node, d455Sensor, D455_IMAGE_TOPIC_MAP);
         loopOnDemand(d455PublishThread, realsensePublishDemandNode);

         initializeHeightMap();
      }
   }

   private void initializeHeightMap()
   {
      boolean runWithCUDA = false;
      heightMapUpdateThread = new RapidHeightMapUpdateThread(ros2Helper,
                                                             syncedRobot,
                                                             syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT),
                                                             syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT),
                                                             d455Sensor,
                                                             RealsenseImageSensor.DEPTH_IMAGE_KEY,
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
