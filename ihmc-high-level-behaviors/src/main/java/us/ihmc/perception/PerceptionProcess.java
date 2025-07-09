package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.HashMap;

public class PerceptionProcess
{
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   HashMap<Integer, ImageSensorPublishThread> sensors = new HashMap<>();

   public PerceptionProcess(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
   }

   public void addD455Sensor(ImageSensor d455Sensor)
   {
      ROS2DemandGraphNode realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      d455Sensor.setSensorFrame(syncedRobot.getReferenceFrames().getSteppingCameraFrame());
      loopOnDemand(d455Sensor.getGrabThread(), realsenseDemandNode);

      ROS2DemandGraphNode realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      realsenseDemandNode.addDependents(realsensePublishDemandNode);

      ImageSensorPublishThread d455PublishThread = new ImageSensorPublishThread(ros2Node, d455Sensor);
      d455PublishThread.addTopic(PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS, RealSenseImageSensor.COLOR_IMAGE_KEY);
      d455PublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(d455PublishThread, realsensePublishDemandNode);
   }

   public void addZED2iSensor(ImageSensor zed2iSensor)
   {
      zed2iSensor.setSensorFrame(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      ROS2DemandGraphNode zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
      zed2iSensor.run(true); // Always start ZED, do not wait for any demand node

      ImageSensorPublishThread zed2iPublishThread = new ImageSensorPublishThread(ros2Node, zed2iSensor);
      zed2iPublishThread.addTopic(PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zed2iPublishThread.addTopic(PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS, ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zed2iPublishThread.addTopic(PerceptionAPI.ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(zed2iPublishThread, zedPublishDemandNode);
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
