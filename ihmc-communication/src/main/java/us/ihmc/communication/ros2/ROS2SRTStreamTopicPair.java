package us.ihmc.communication.ros2;

import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.ros2.ROS2Topic;

/**
 * Pair of an SRT stream topic and an image message topic.
 * Used to identify which image message topic to relay video streams on.
 * @param streamStatusTopic The SRT stream topic.
 * @param imageMessageTopic The matching image message topic.
 * @param isDepth Whether the image is a depth image.
 */
public record ROS2SRTStreamTopicPair(ROS2Topic<SRTStreamStatus> streamStatusTopic, ROS2Topic<ImageMessage> imageMessageTopic, boolean isDepth) {}
