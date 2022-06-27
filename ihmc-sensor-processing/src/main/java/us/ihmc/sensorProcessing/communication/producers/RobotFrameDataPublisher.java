package us.ihmc.sensorProcessing.communication.producers;

import controller_msgs.msg.dds.RobotFrameData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class RobotFrameDataPublisher
{
   //   private ReferenceFrame referenceFrame;

   private IHMCRealtimeROS2Publisher<RobotFrameData> ros2Publisher;

   private RobotFrameData robotFrameData = new RobotFrameData();

   public RobotFrameDataPublisher(ReferenceFrame referenceFrame, RealtimeROS2Node realtimeROS2Node, ROS2Topic<?> outputTopic)
   {
      robotFrameData.getFrameName().add(referenceFrame.getName());
      robotFrameData.getFramePoseInWorld().set(referenceFrame.getTransformToWorldFrame());
      ROS2Topic<?> ros2Topic = outputTopic.withSuffix("FrameData");

      ros2Publisher = ROS2Tools.createPublisher(realtimeROS2Node, RobotFrameData.class, ros2Topic);
      System.out.println("frame topic: " + ros2Topic.toString());
   }

   public boolean publish()
   {
      return ros2Publisher.publish(robotFrameData);
   }
}
