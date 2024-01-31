package us.ihmc.communication.ros2;

/**
 * This class is an enum for the concept of "command" and "status"
 * channels as part of a ROS 2 module or service's API.
 *
 * With ROS 2, to have bidriectional communication for a topic of a
 * type, you must add uniqueness to the topic name between the directions.
 * For example /module/input and /module/output
 * This is because if you only had one topic, /module, publishing would
 * also trigger your onw subscription callback. Instead we want the messages
 * to to only go to the other side.
 *
 * "input" and "output", doesn't work well as one side would be publishing
 * on the "input" topic, so we give the concept of the "actor" to one side,
 * where it accepts "commands" and gives "statuses".
 *
 * See also {@link ROS2IOTopicPair}.
 */
public enum ROS2IOTopicQualifier
{
   COMMAND,
   STATUS;

   public ROS2IOTopicQualifier getOpposite()
   {
      return this == COMMAND ? STATUS : COMMAND;
   }
}
