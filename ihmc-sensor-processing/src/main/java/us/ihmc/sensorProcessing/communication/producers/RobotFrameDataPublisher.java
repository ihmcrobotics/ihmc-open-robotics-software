package us.ihmc.sensorProcessing.communication.producers;

import controller_msgs.msg.dds.RobotFrameData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * ROS2 Publisher for {@link RobotFrameData}
 * <p>
 * It is called automatically when {@link RobotConfigurationDataPublisher} is created from
 * {@link RobotConfigurationDataPublisherFactory}.
 * </p>
 */
public class RobotFrameDataPublisher
{
   private IHMCRealtimeROS2Publisher<RobotFrameData> ros2Publisher;

   private RobotFrameData robotFrameData = new RobotFrameData();

   private ReferenceFrame myReferenceFrame;

   public RobotFrameDataPublisher(ReferenceFrame referenceFrame, RealtimeROS2Node realtimeROS2Node, ROS2Topic<?> outputTopic)
   {
      myReferenceFrame = referenceFrame;
      robotFrameData.getFrameName().add(referenceFrame.getName());

      ROS2Topic<?> ros2Topic = outputTopic.withSuffix(referenceFrame.getName());

      ros2Publisher = ROS2Tools.createPublisher(realtimeROS2Node, RobotFrameData.class, ros2Topic);
   }

   public boolean publish()
   {
      robotFrameData.getFramePoseInWorld().set(myReferenceFrame.getTransformToWorldFrame());
      return ros2Publisher.publish(robotFrameData);
   }
}