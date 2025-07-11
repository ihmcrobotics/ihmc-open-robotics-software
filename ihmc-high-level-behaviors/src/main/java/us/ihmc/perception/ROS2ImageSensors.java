package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class ROS2ImageSensors
{
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;

   // Realsense
   private ROS2DemandGraphNode realsenseDemandNode;
   private ROS2DemandGraphNode realsensePublishDemandNode;
   private ImageSensorPublishThread realsensePublishThread;

   // ZED
   private ROS2DemandGraphNode zedPublishDemandNode;
   private ImageSensorPublishThread zedPublishThread;

   public ROS2ImageSensors(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
   }

   public void addRealsenseSensor(ImageSensor realsenseSensor)
   {
      realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      realsenseSensor.setSensorFrame(syncedRobot.getReferenceFrames().getSteppingCameraFrame());
      setupCallbackForDemandNode(realsenseSensor.getGrabThread(), realsenseDemandNode);

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      realsenseDemandNode.addDependents(realsensePublishDemandNode);

      realsensePublishThread = new ImageSensorPublishThread(ros2Node, realsenseSensor);
      realsensePublishThread.addTopic(PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS, RealSenseImageSensor.COLOR_IMAGE_KEY);
      realsensePublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      setupCallbackForDemandNode(realsensePublishThread, realsensePublishDemandNode);
   }

   public void addZEDSensor(ImageSensor zedSensor)
   {
      zedSensor.setSensorFrame(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
      zedSensor.run(true); // Always start ZED, do not wait for any demand node

      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSensor);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS, ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      setupCallbackForDemandNode(zedPublishThread, zedPublishDemandNode);
   }

   private static void setupCallbackForDemandNode(RepeatingTaskThread loopThread, ROS2DemandGraphNode demandNode)
   {
      if (!loopThread.isAlive())
         loopThread.start();

      if (demandNode.isDemanded())
         loopThread.startRepeating();

      demandNode.addDemandChangedCallback(loopThread::setRepeating);
   }

   public void destroy()
   {
      // Destroy Realsense
      realsenseDemandNode.destroy();
      realsensePublishDemandNode.destroy();
      realsensePublishThread.blockingKill();

      // Destroy ZED
      zedPublishDemandNode.destroy();
      zedPublishThread.blockingKill();
   }
}
