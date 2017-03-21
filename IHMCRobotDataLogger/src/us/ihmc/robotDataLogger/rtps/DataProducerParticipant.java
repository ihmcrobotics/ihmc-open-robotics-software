package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.io.InputStream;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.DurabilityKind;
import us.ihmc.pubsub.attributes.HistoryQosPolicy.HistoryQosPolicyKind;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.attributes.TopicAttributes.TopicKind;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.robotDataLogger.ModelFileDescription;
import us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType;
import us.ihmc.tools.thread.ThreadTools;

public class DataProducerParticipant
{
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   
   private final String guidString;
   
   
   public DataProducerParticipant(String name) throws IOException
   {
      ParticipantAttributes<?> att = domain.createParticipantAttributes();
      att.setDomainId(LogParticipantSettings.domain);
      att.setLeaseDuration(Time.Infinite);
      att.setName(name);
      participant = domain.createParticipant(att, null);
      
      Guid guid = participant.getGuid();
      
      StringBuilder guidBuilder = new StringBuilder();
      guidBuilder.append("{");
      for(byte val : guid.getGuidPrefix().getValue())
      {
         guidBuilder.append(String.format("%02x", val));
      }
//      guidBuilder.append("-");
//      for(byte val : guid.getEntity().getValue())
//      {
//         guidBuilder.append(String.format("%02x", val));
//      }
      guidBuilder.append("}");
      guidString = guidBuilder.toString();
      
   }
   
   private String getUniquePartition()
   {
      return LogParticipantSettings.partition + LogParticipantSettings.namespaceSeperator + guidString;
   }
   
   private PublisherAttributes<?, ?> createPersistantPublisherAttributes(TopicDataType<?> topicDataType, String partition, String topicName)
   {
      domain.registerType(participant, topicDataType);
      
      PublisherAttributes<?,?> publisherAttributes = domain.createPublisherAttributes();
      publisherAttributes.getTopic().setTopicKind(TopicKind.NO_KEY);
      publisherAttributes.getTopic().setTopicDataType(topicDataType.getName());
      publisherAttributes.getTopic().setTopicName(topicName);
      publisherAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
      publisherAttributes.getQos().addPartition(partition);
      
      publisherAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setKind(HistoryQosPolicyKind.KEEP_LAST_HISTORY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setDepth(1); 
      
      return publisherAttributes;
   }
   
   public void setModelFileProvider(LogModelProvider provider) throws IOException
   {
      ModelFileDescriptionPubSubType topicDataType = new ModelFileDescriptionPubSubType();
      String partition = getUniquePartition();
      String topicName = "modelFile";
      
      PublisherAttributes<?, ?> att = createPersistantPublisherAttributes(topicDataType, partition, topicName);
      Publisher publisher = domain.createPublisher(participant, att, null);
      
      ModelFileDescription modelFileDescription = new ModelFileDescription();
      modelFileDescription.setName(provider.getModelName());
      for(String resourceDirectory : provider.getResourceDirectories())
      {
         modelFileDescription.getResourceDirectories().add(resourceDirectory);
      }

      publisher.write(modelFileDescription);
   }
   
   public static void main(String[] args) throws IOException
   {
      DataProducerParticipant participant = new DataProducerParticipant("testParticipant");
      InputStream testStream = DataProducerParticipant.class.getClassLoader().getResourceAsStream("Models/unitBox.sdf");
      String[] resourcesDirectories = { "Models" };
      LogModelProvider logModelProvider = new SDFLogModelProvider("testModel", testStream, resourcesDirectories);
      
      participant.setModelFileProvider(logModelProvider);
      
      ThreadTools.sleepForever();
   }
  
   
}
