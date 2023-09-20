package us.ihmc.communication.ros2;

public enum ROS2IOTopicQualifier
{
   COMMAND,
   STATUS;

   public ROS2IOTopicQualifier getOpposite()
   {
      return this == COMMAND ? STATUS : COMMAND;
   }
}
