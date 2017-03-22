package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.io.InputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.DurabilityKind;
import us.ihmc.pubsub.attributes.HistoryQosPolicy.HistoryQosPolicyKind;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.PublishModeKind;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.pubsub.attributes.TopicAttributes.TopicKind;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.AnnouncementPubSubType;
import us.ihmc.robotDataLogger.CameraAnnouncement;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.tools.thread.ThreadTools;

/**
 * This class implements all communication for a data producer inside a DDS logging network
 * 
 * @author jesper
 *
 */
public class DataProducerParticipant
{
   private final String name;
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private final String guidString;
   private boolean activated = false;
   
   private Handshake handshake;
   private LogModelProvider logModelProvider;
   private ArrayList<CameraAnnouncement> cameras = new ArrayList<>();
   private InetAddress dataAddress;
   private int dataPort;
   private boolean log = false;

   private DataProducerListener dataProducerListener = null;
   
   private class VariableChangeSubscriberListener implements SubscriberListener
   {

      @Override
      public void onNewDataMessage(Subscriber subscriber)
      {
         VariableChangeRequest msg = new VariableChangeRequest();
         SampleInfo info = new SampleInfo();
         try
         {
            if(subscriber.takeNextData(msg, info))
            {
               if(dataProducerListener != null)
               {
                  dataProducerListener.changeVariable(msg.getVariableID(), msg.getRequestedValue());
               }
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      @Override
      public void onSubscriptionMatched(Subscriber subscriber, MatchingInfo info)
      {
         
      }
      
   }
   
   
   public DataProducerParticipant(String name) throws IOException
   {
      this.name = name;
      ParticipantAttributes<?> att = domain.createParticipantAttributes();
      att.setDomainId(LogParticipantSettings.domain);
      att.setLeaseDuration(Time.Infinite);
      att.setName(name);
      participant = domain.createParticipant(att, null);

      guidString = LogParticipantTools.createGuidString(participant.getGuid());
      
   }
   
   /**
    * Set the data producer IP address for streaming data 
    * 
    * Temporary till streaming is done over RTPS
    * 
    * Required
    * 
    * @param dataAddress
    */
   @Deprecated
   public void setDataAddress(InetAddress dataAddress)
   {
      this.dataAddress = dataAddress;
   }

   /** 
    * Set the data producer port for streaming data
    * 
    * Temporary till streaming is done over RTPS
    * 
    * Required
    * 
    * @param port
    */
   @Deprecated
   public void setPort(int port)
   {
      this.dataPort = port;
   }

   /**
    * Add a callback listener for variable change requests
    * 
    * Required
    * 
    * @param listener
    */
   public void setDataProducerListener(DataProducerListener listener)
   {
      this.dataProducerListener = listener;
   }
   
   /**
    * Set the handshake data
    * 
    * Required
    * 
    * @param handshake
    */
   public void setHandshake(Handshake handshake)
   {
      this.handshake = handshake;
   }

   private String getUniquePartition()
   {
      return LogParticipantSettings.partition + LogParticipantSettings.namespaceSeperator + guidString;
   }

   private PublisherAttributes<?, ?> createPersistentPublisherAttributes(TopicDataType<?> topicDataType, String partition, String topicName)
   {
      domain.registerType(participant, topicDataType);

      PublisherAttributes<?, ?> publisherAttributes = domain.createPublisherAttributes();
      publisherAttributes.getTopic().setTopicKind(TopicKind.NO_KEY);
      publisherAttributes.getTopic().setTopicDataType(topicDataType.getName());
      publisherAttributes.getTopic().setTopicName(topicName);
      publisherAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
      publisherAttributes.getQos().addPartition(partition);

      publisherAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setKind(HistoryQosPolicyKind.KEEP_LAST_HISTORY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setDepth(1);
      
      if(topicDataType.getTypeSize() > 65000)
      {
         publisherAttributes.getQos().setPublishMode(PublishModeKind.ASYNCHRONOUS_PUBLISH_MODE);
      }

      return publisherAttributes;
   }

   private <T> void publishPersistentData(String partition, String topicName, TopicDataType<T> topicDataType, T data) throws IOException
   {
      PublisherAttributes<?, ?> att = createPersistentPublisherAttributes(topicDataType, partition, topicName);
      Publisher publisher = domain.createPublisher(participant, att, null);
      publisher.write(data);
   }

   /**
    * Set a model provider for this logger
    * 
    * Optional
    * 
    * @param provider
    * @throws IOException
    */
   public void setModelFileProvider(LogModelProvider provider) throws IOException
   {
      this.logModelProvider = provider;
   }
   
   /** 
    * Add cameras to log 
    * 
    * Optional
    * 
    * @param name User friendly name to show in the log files
    * @param cameraId ID of the camera on the logger machine
    */
   public void addCamera(String name, int cameraId)
   {
      CameraAnnouncement cameraAnnouncement = new CameraAnnouncement();
      cameraAnnouncement.setName(name);
      cameraAnnouncement.setId(cameraId);
      cameras.add(cameraAnnouncement);
   }
   
   

   /**
    * Activate the producer. This will publish the model, handshake and logger announcement to the logger dds network
    * 
    * @throws IOException
    */
   public void activate() throws IOException
   {
      if (activated)
      {
         throw new IOException("This participant is already activated.");
      }
      if(dataAddress == null || dataPort < 1024)
      {
         throw new RuntimeException("No data address and valid port (>=1024) provided");
      }
      if(handshake == null)
      {
         throw new RuntimeException("No handshake provided");
      }
      
      String partition = getUniquePartition();

      AnnouncementPubSubType announcementPubSubType = new AnnouncementPubSubType();
      Announcement announcement = new Announcement();
      
      announcement.setName(name);
      announcement.setIdentifier(guidString);
      System.arraycopy(dataAddress.getAddress(), 0, announcement.getDataIP(), 0, 4);
      announcement.setDataPort((short) dataPort);
      announcement.setLog(log);

      HandshakePubSubType handshakePubSubType = new HandshakePubSubType();
      System.out.println(handshakePubSubType.getTypeSize());
      publishPersistentData(partition, LogParticipantSettings.handshakeTopic, handshakePubSubType, handshake);
      
      if (logModelProvider != null)
      {
         announcement.getModelFileDescription().setHasModel(true);
         announcement.getModelFileDescription().setName(logModelProvider.getModelName());
         announcement.getModelFileDescription().setModelFileSize(logModelProvider.getModel().length);
         for (String resourceDirectory : logModelProvider.getResourceDirectories())
         {
            announcement.getModelFileDescription().getResourceDirectories().add(resourceDirectory);
         }

         ByteBufferPubSubType modelFilePubSubType = new ByteBufferPubSubType("us::ihmc::robotDataLogger::modelFile", logModelProvider.getModel().length);
         ByteBuffer modelBuffer = ByteBuffer.wrap(logModelProvider.getModel());
         publishPersistentData(partition, LogParticipantSettings.modelFileTopic, modelFilePubSubType, modelBuffer);

         if (logModelProvider.getResourceZip() != null && logModelProvider.getResourceZip().length > 0)
         {
            ByteBufferPubSubType resourcesPubSubType = new ByteBufferPubSubType("us::ihmc::robotDataLogger::modelFile", logModelProvider.getResourceZip().length);
            ByteBuffer resourceBuffer = ByteBuffer.wrap(logModelProvider.getResourceZip());
            publishPersistentData(partition, LogParticipantSettings.resourceBundleTopic, resourcesPubSubType, resourceBuffer);

            announcement.getModelFileDescription().setHasResourceZip(true);
            announcement.getModelFileDescription().setResourceZipSize(logModelProvider.getResourceZip().length);
         }
      }
      else
      {
         announcement.getModelFileDescription().setHasModel(false);
      }
      
      for(CameraAnnouncement camera : cameras)
      {
         announcement.getCameras().add().set(camera);
      }
      
      if(dataProducerListener != null)
      {
         VariableChangeRequestPubSubType variableChangeRequestPubSubType = new VariableChangeRequestPubSubType();
         domain.registerType(participant, variableChangeRequestPubSubType);
         SubscriberAttributes<?, ?> subscriberAttributes = domain.createSubscriberAttributes();
         subscriberAttributes.getTopic().setTopicKind(TopicKind.NO_KEY);
         subscriberAttributes.getTopic().setTopicDataType(variableChangeRequestPubSubType.getName());
         subscriberAttributes.getTopic().setTopicName(LogParticipantSettings.variableChangeTopic);
         subscriberAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
         subscriberAttributes.getQos().addPartition(partition);
         domain.createSubscriber(participant, subscriberAttributes, new VariableChangeSubscriberListener());

      }
      
      publishPersistentData(LogParticipantSettings.partition, LogParticipantSettings.annoucementTopic, announcementPubSubType, announcement);

   }

   /**
    * Set if the logger should record this session or not
    * 
    * @param log
    */
   public void setLog(boolean log)
   {
      this.log = log;
   }

   public static void main(String[] args) throws IOException
   {
      DataProducerParticipant participant = new DataProducerParticipant("testParticipant");
      InputStream testStream = DataProducerParticipant.class.getClassLoader().getResourceAsStream("Models/unitBox.sdf");
      String[] resourcesDirectories = {"Models"};
      LogModelProvider logModelProvider = new SDFLogModelProvider("testModel", testStream, resourcesDirectories);

      
      participant.setDataAddress(Inet4Address.getLocalHost());
      participant.setPort(2048);
      participant.setModelFileProvider(logModelProvider);
      participant.setHandshake(new Handshake());
      participant.setDataProducerListener(new DataProducerListener()
      {
         
         @Override
         public void changeVariable(int id, double newValue)
         {
            System.out.println("Required change for " + id + " to " + newValue);
         }
      });
      
      participant.activate();

      ThreadTools.sleepForever();
   }

}
