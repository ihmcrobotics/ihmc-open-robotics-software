package us.ihmc.avatar.ros2.visualizer;

import us.ihmc.pubsub.attributes.QosInterface;

public interface SCSROS2VisualizerTopicAttributes
{
   String getTopicName();

   String getTopicType();

   QosInterface getQosInterface();
}
