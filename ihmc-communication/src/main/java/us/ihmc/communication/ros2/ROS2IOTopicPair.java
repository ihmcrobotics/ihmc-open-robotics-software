package us.ihmc.communication.ros2;

import us.ihmc.ros2.ROS2Topic;

public class ROS2IOTopicPair<T>
{
   private final ROS2Topic<T> inputTopic;
   private final ROS2Topic<T> outputTopic;

   public ROS2IOTopicPair(ROS2Topic<T> baseTopic)
   {
      inputTopic = baseTopic.withInput();
      outputTopic = baseTopic.withOutput();
   }

   public ROS2Topic<T> getInputTopic()
   {
      return inputTopic;
   }

   public ROS2Topic<T> getOutputTopic()
   {
      return outputTopic;
   }
}
