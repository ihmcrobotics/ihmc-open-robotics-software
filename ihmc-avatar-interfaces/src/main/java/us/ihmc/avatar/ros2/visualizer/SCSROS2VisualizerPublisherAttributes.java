package us.ihmc.avatar.ros2.visualizer;

public class SCSROS2VisualizerPublisherAttributes implements SCSROS2VisualizerTopicAttributes
{
   private final String topicName;
   private final String topicType;

   public SCSROS2VisualizerPublisherAttributes(String typeName, String topicName)
   {
      this.topicName = topicName;
      this.topicType = typeName;
   }

   @Override
   public String getTopicName()
   {
      return null;
   }

   @Override
   public String getTopicType()
   {
      return null;
   }
}
