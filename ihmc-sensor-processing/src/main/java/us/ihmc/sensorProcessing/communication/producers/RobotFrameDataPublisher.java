package us.ihmc.sensorProcessing.communication.producers;

import ihmc_common_msgs.msg.dds.RobotFrameData;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private final ROS2PublisherBasics<RobotFrameData> ros2Publisher;
   private final RobotFrameData robotFrameData = new RobotFrameData();
   private final ReferenceFrame myReferenceFrame;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RobotFrameDataPublisher(ReferenceFrame referenceFrame, RealtimeROS2Node realtimeROS2Node, ROS2Topic<?> outputTopic)
   {
      myReferenceFrame = referenceFrame;
      robotFrameData.getFrameName().append(referenceFrame.getName());

      ROS2Topic<RobotFrameData> ros2Topic = outputTopic.withSuffix(referenceFrame.getName()).withType(RobotFrameData.class);

      ros2Publisher = realtimeROS2Node.createPublisher(ros2Topic);
   }

   public boolean publish()
   {
      myReferenceFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      robotFrameData.getFramePoseInWorld().set(tempTransform);
      return ros2Publisher.publish(robotFrameData);
   }

   public static ROS2Topic<RobotFrameData> getTopic(String robotName, ReferenceFrame referenceFrame)
   {
      return getTopic(robotName, referenceFrame.getName());
   }

   public static ROS2Topic<RobotFrameData> getTopic(String robotName, String referenceFrameName)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName).withType(RobotFrameData.class).withSuffix(referenceFrameName);
   }
}