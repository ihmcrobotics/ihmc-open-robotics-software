package us.ihmc.perception;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.concurrent.BlockingQueue;

public class ROS2ImageSensors
{
   private final ROS2Node ros2Node;

   // Realsense
   private ImageSensor realsenseSensor;
   private ROS2DemandGraphNode realsenseDemandNode;
   private ROS2DemandGraphNode realsensePublishDemandNode;
   private ImageSensorPublishThread realsensePublishThread;

   // ZED
   private ImageSensor zedSensor;
   private ROS2DemandGraphNode zedPublishDemandNode;
   private ImageSensorPublishThread zedPublishThread;
   private ROS2DemandGraphNode ros2DemandGraphNode;
   private ImageSensorPublishThread imageSensorPublishThread;

   public ROS2ImageSensors(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void addRealsenseSensor(ImageSensor realsenseSensor, ReferenceFrame sensorFrame)
   {
      this.realsenseSensor = realsenseSensor;
      realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      realsenseSensor.setSensorFrame(sensorFrame);
      realsenseSensor.run(true);

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      realsenseDemandNode.addDependents(realsensePublishDemandNode);

      realsensePublishThread = new ImageSensorPublishThread(ros2Node, realsenseSensor);
      realsensePublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE, RealSenseImageSensor.DEPTH_IMAGE_KEY, 0.25);
      realsensePublishThread.addTopic(PerceptionAPI.D455_COLOR_IMAGE, RealSenseImageSensor.COLOR_IMAGE_KEY, 0.25);
      setupCallbackForDemandNode(realsensePublishThread, realsensePublishDemandNode);
   }

   // Must be called after the realsense has been added, no safety checks
   public void addFullResolutionRealSensePublisher(ImageSensor realsenseSensor)
   {
      ros2DemandGraphNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_FULL_RESOLUTION_HEARTBEAT);
      realsenseDemandNode.addDependents(ros2DemandGraphNode);

      imageSensorPublishThread = new ImageSensorPublishThread(ros2Node, realsenseSensor);
      realsensePublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE_FULL_RESOLUTION, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      realsensePublishThread.addTopic(PerceptionAPI.D455_COLOR_IMAGE_FULL_RESOLUTION, RealSenseImageSensor.COLOR_IMAGE_KEY);
      setupCallbackForDemandNode(imageSensorPublishThread, ros2DemandGraphNode);
   }

   public void addZEDSensor(ImageSensor zedSensor, ReferenceFrame sensorFrame)
   {
      this.zedSensor = zedSensor;
      zedSensor.setSensorFrame(sensorFrame);
      zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
      zedSensor.run(true); // Always start ZED, do not wait for any demand node

      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSensor);
      zedPublishThread.addTopic(PerceptionAPI.ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
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

   /**
    * These could all be null because we can't guarantee which sensors will be created.
    * This is because we know about the sensors that can exist on the humanoid robots, but we can't guarantee that both sensors are created for each robot.
    */
   public void destroy()
   {
      // Destroy Realsense
      if (realsenseSensor != null)
         realsenseSensor.close();
      if (realsenseDemandNode != null)
         realsenseDemandNode.destroy();
      if (realsensePublishDemandNode != null)
         realsensePublishDemandNode.destroy();
      if (realsensePublishThread != null)
         realsensePublishThread.blockingKill();

      // Destroy ZED
      if (zedSensor != null)
         zedSensor.close();
      if  (zedPublishDemandNode != null)
         zedPublishDemandNode.destroy();
      if (zedPublishThread != null)
         zedPublishThread.blockingKill();
   }

   /**
    * We are attempting to add this queue holder to our image sensor.
    * That sensor could be null depending on the robot you are using.
    * Having the null check allows for the underlying algorithm to stay the same regardless of what sensors are being used.
    */
   public void registerImageQueueForRealsense(BlockingQueue<RawImage> rawImageCollection, int imageKey)
   {
      if (realsenseSensor != null)  // Don't have a Realsense Sensor created for this robot
         realsenseSensor.registerImageQueue(rawImageCollection, imageKey);
   }
   /**
    * We are attempting to add this queue holder to our image sensor.
    * That sensor could be null depending on the robot you are using.
    * Having the null check allows for the underlying algorithm to stay the same regardless of what sensors are being used.
    */
   public void registerImageQueueForZED(BlockingQueue<RawImage> rawImageCollection, int imageKey)
   {
      if (zedSensor != null)  // Don't have a ZED Sensor created for this robot
         zedSensor.registerImageQueue(rawImageCollection, imageKey);
   }
}
