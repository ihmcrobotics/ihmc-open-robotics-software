package us.ihmc.sensorProcessing.communication.producers;

import ihmc_common_msgs.RobotFrameData;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.ROS2Node;

/**
 * ROS2 Publisher for {@link RobotFrameData}
 * <p>
 * It is called automatically when {@link RobotConfigurationDataPublisher} is created from
 * {@link RobotConfigurationDataPublisherFactory}.
 * </p>
 */
public class RobotFrameDataPublisher
{
   public static final boolean ENABLE_ROBOT_FRAME_DATA_PUBLISHERS = false;

   private final ROS2Publisher<RobotFrameData> ros2Publisher;
   private final RobotFrameData robotFrameData = new RobotFrameData();
   private final ReferenceFrame myReferenceFrame;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RobotFrameDataPublisher(ReferenceFrame referenceFrame, ROS2Node ros2Node, ROS2Topic<?> outputTopic)
   {
      myReferenceFrame = referenceFrame;
      robotFrameData.getFrameName().append(referenceFrame.getName());

      if (ENABLE_ROBOT_FRAME_DATA_PUBLISHERS)
      {
         ROS2Topic<RobotFrameData> ros2Topic = outputTopic.appendedWith(referenceFrame.getName()).withType(RobotFrameData.class);
         ros2Publisher = ros2Node.createPublisher(ros2Topic);
      }
      else
      {
         ros2Publisher = null;
      }
   }

   public boolean publish()
   {
      if (ENABLE_ROBOT_FRAME_DATA_PUBLISHERS)
      {
         myReferenceFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
         robotFrameData.getFramePoseInWorld().getPose().set(tempTransform);
         ros2Publisher.publish(robotFrameData);
         return true;
      }
      else
      {
         return true;
      }
   }

   public static ROS2Topic<RobotFrameData> getTopic(String robotName, ReferenceFrame referenceFrame)
   {
      return getTopic(robotName, referenceFrame.getName());
   }

   public static ROS2Topic<RobotFrameData> getTopic(String robotName, String referenceFrameName)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName).withType(RobotFrameData.class).appendedWith(referenceFrameName);
   }
}