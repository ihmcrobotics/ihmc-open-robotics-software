package us.ihmc.avatar.ros.visualizer;

import us.ihmc.pubsub.attributes.QosInterface;

public interface SCSROS2VisualizerTopicAttributes
{
   String getTopicName();

   String getTopicType();

   QosInterface getQosInterface();
}
