package us.ihmc.avatar.ros.visualizer;

import javafx.application.Platform;
import us.ihmc.pubsub.common.Guid;

import java.util.HashMap;

public class SCSROS2VisualizerPartition
{
   private static final String DEFAULT_PARTITION = "[Default partition]";
   private final String name;
   private final HashMap<String, SCSROS2VisualizerTopic> topics = new HashMap<>();

   public SCSROS2VisualizerPartition()
   {
      this(DEFAULT_PARTITION);
   }

   public SCSROS2VisualizerPartition(String name)
   {
      this.name = name;
   }

   private SCSROS2VisualizerTopic getHolder(String topicName)
   {
      return topics.computeIfAbsent(topicName, key -> new SCSROS2VisualizerTopic(key, name));
   }

   public void addPublisher(Guid guid, SCSROS2Participant participantHolder, SCSROS2VisualizerPublisherAttributes attributes)
   {
//      SCSROS2VisualizerTopic holder = getHolder(attributes.getTopicName());
//      holder.addPublisher(guid, participantHolder, attributes);
   }

}
