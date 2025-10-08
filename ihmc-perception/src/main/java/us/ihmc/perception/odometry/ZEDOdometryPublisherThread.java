package us.ihmc.perception.odometry;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class ZEDOdometryPublisherThread extends RepeatingTaskThread
{
   private final ZEDImageSensor zedSensor;
   private final ROS2Publisher<RigidBodyTransformMessage> posePublisher;
   
   public ZEDOdometryPublisherThread(ROS2Node ros2Node, ZEDImageSensor zedSensor)
   {
      super(zedSensor.getSensorName() + "OdometryPublishThread");
      this.zedSensor = zedSensor;
      this.posePublisher = ros2Node.createPublisher(PerceptionAPI.ZED_TRACKED_POSE);
   }

   @Override
   protected void runTask()
   {
      // Example: get pose from ZED sensor, build message, publish
      RigidBodyTransformMessage poseMsg = new RigidBodyTransformMessage();
      MessageTools.toMessage(zedSensor.getTrackedSensorFrame().getTransformToRoot(), poseMsg);
      posePublisher.publish(poseMsg);
   }
   
   @Override
   public void kill()
   {
      super.kill();
      interrupt();
   }
}