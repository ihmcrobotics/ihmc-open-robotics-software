package us.ihmc.communication;

@Deprecated
public enum ROS2TopicQualifier
{
   @Deprecated
   INPUT(ROS2Tools.INPUT_TOPIC_QUALIFIER),
   @Deprecated
   OUTPUT(ROS2Tools.OUTPUT_TOPIC_QUALIFIER);

   private final String name;

   @Deprecated
   ROS2TopicQualifier(String name)
   {
      this.name = name;
   }

   @Override
   public String toString()
   {
      return name;
   }
}
