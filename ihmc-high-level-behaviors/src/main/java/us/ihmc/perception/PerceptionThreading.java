package us.ihmc.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class PerceptionThreading
{
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;

   public PerceptionThreading(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
   }

   public void addRealsenseSensor(ImageSensor realsenseSensor)
   {
      ROS2DemandGraphNode realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      realsenseSensor.setSensorFrame(syncedRobot.getReferenceFrames().getSteppingCameraFrame());
      loopOnDemand(realsenseSensor.getGrabThread(), realsenseDemandNode);

      ROS2DemandGraphNode realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      realsenseDemandNode.addDependents(realsensePublishDemandNode);

      ImageSensorPublishThread realsensePublishThread = new ImageSensorPublishThread(ros2Node, realsenseSensor);
      realsensePublishThread.addTopic(PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS, RealSenseImageSensor.COLOR_IMAGE_KEY);
      realsensePublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(realsensePublishThread, realsensePublishDemandNode);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         realsenseDemandNode.destroy();
                                                         realsensePublishDemandNode.destroy();
                                                         realsensePublishThread.blockingKill();
                                                      }));
   }

   public void addZEDSensor(ImageSensor zedSensor)
   {
      zedSensor.setSensorFrame(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      ROS2DemandGraphNode zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
      zedSensor.run(true); // Always start ZED, do not wait for any demand node

      ImageSensorPublishThread zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSensor);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS, ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(zedPublishThread, zedPublishDemandNode);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         zedPublishDemandNode.destroy();
                                                         zedPublishThread.blockingKill();
                                                      }));
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
