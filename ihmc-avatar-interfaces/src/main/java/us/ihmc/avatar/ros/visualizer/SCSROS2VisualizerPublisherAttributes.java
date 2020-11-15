package us.ihmc.avatar.ros.visualizer;

import us.ihmc.pubsub.attributes.QosInterface;
import us.ihmc.pubsub.attributes.WriterQosHolder;

public class SCSROS2VisualizerPublisherAttributes implements SCSROS2VisualizerTopicAttributes
{
   private final String topicName;
   private final String topicType;
   private final WriterQosHolder writerQosHolder;

   public SCSROS2VisualizerPublisherAttributes(String typeName, String topicName, WriterQosHolder writerQosHolder)
   {
      this.topicName = topicName;
      this.topicType = typeName;
      this.writerQosHolder = writerQosHolder;
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

   @Override
   public QosInterface getQosInterface()
   {
      return null;
   }
}
