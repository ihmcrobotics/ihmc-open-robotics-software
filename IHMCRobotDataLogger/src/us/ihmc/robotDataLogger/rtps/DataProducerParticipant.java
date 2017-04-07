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
import us.ihmc.pubsub.common.LogLevel;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.AnnouncementPubSubType;
import us.ihmc.robotDataLogger.CameraAnnouncement;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.Timestamp;
import us.ihmc.robotDataLogger.TimestampPubSubType;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.rtps.impl.fastRTPS.WriterTimes;
import us.ihmc.tools.thread.ThreadTools;

/**
 * This class implements all communication for a data producer inside a DDS logging network
 * 
 * @author Jesper Smith
 *
 */
public class DataProducerParticipant
{
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private final String guidString;
   private volatile boolean activated = false;

   private ArrayList<CameraAnnouncement> cameras = new ArrayList<>();
   private InetAddress dataAddress;
   private int dataPort;
   private boolean log = false;

   private Handshake handshake;
   private final Announcement announcement = new Announcement();
   private final Timestamp timestamp = new Timestamp();

   private final Publisher timestampPublisher;
   private final Publisher announcementPublisher;
   private final Publisher handshakePublisher;

   private final VariableChangedListener dataProducerListener;

   private class VariableChangeSubscriberListener implements SubscriberListener
   {

      @Override
      public void onNewDataMessage(Subscriber subscriber)
      {
         VariableChangeRequest msg = new VariableChangeRequest();
         SampleInfo info = new SampleInfo();
         try
         {
            if (subscriber.takeNextData(msg, info))
            {
               if (dataProducerListener != null && activated)
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

   public DataProducerParticipant(String name, LogModelProvider logModelProvider, VariableChangedListener variableChangedListener) throws IOException
   {
      announcement.setName(name);
      this.dataProducerListener = variableChangedListener;

      domain.setLogLevel(LogLevel.ERROR);
      ParticipantAttributes att = domain.createParticipantAttributes(LogParticipantSettings.domain, name);
      participant = domain.createParticipant(att);

      guidString = LogParticipantTools.createGuidString(participant.getGuid());

      String partition = getUniquePartition();
      handshakePublisher = createPersistentPublisher(partition, LogParticipantSettings.handshakeTopic, new HandshakePubSubType());
      announcementPublisher = createPersistentPublisher(LogParticipantSettings.partition, LogParticipantSettings.annoucementTopic,
                                                        new AnnouncementPubSubType());

      if (logModelProvider != null)
      {
         byte[] model = logModelProvider.getModel();
         ByteBufferPubSubType modelFilePubSubType = new ByteBufferPubSubType(LogParticipantSettings.modelFileTypeName, model.length);
         announcement.getModelFileDescription().setHasModel(true);
         announcement.getModelFileDescription().setName(logModelProvider.getModelName());
         announcement.getModelFileDescription().setModelLoaderClass(logModelProvider.getLoader().getCanonicalName());
         announcement.getModelFileDescription().setModelFileSize(model.length);
         for (String resourceDirectory : logModelProvider.getResourceDirectories())
         {
            announcement.getModelFileDescription().getResourceDirectories().add(resourceDirectory);
         }

         ByteBuffer modelBuffer = ByteBuffer.wrap(model);
         Publisher logModelFilePublisher = createPersistentPublisher(partition, LogParticipantSettings.modelFileTopic, modelFilePubSubType);
         logModelFilePublisher.write(modelBuffer);

         byte[] resourceZip = logModelProvider.getResourceZip();
         if (resourceZip != null && resourceZip.length > 0)
         {
            ByteBufferPubSubType resourcesPubSubType = new ByteBufferPubSubType(LogParticipantSettings.resourceBundleTypeName, resourceZip.length);
            ByteBuffer resourcesBuffer = ByteBuffer.wrap(resourceZip);
            Publisher resourcesPublisher = createPersistentPublisher(partition, LogParticipantSettings.resourceBundleTopic, resourcesPubSubType);
            resourcesPublisher.write(resourcesBuffer);

            announcement.getModelFileDescription().setHasResourceZip(true);
            announcement.getModelFileDescription().setResourceZipSize(resourceZip.length);

         }
      }
      else
      {
         announcement.getModelFileDescription().setHasModel(false);
      }

      TimestampPubSubType timestampPubSubType = new TimestampPubSubType();
      PublisherAttributes timestampPublisherAttributes = domain.createPublisherAttributes(participant, timestampPubSubType,
                                                                                          LogParticipantSettings.timestampTopic, ReliabilityKind.BEST_EFFORT,
                                                                                          partition);
      timestampPublisher = domain.createPublisher(participant, timestampPublisherAttributes);

      if (dataProducerListener != null)
      {
         VariableChangeRequestPubSubType variableChangeRequestPubSubType = new VariableChangeRequestPubSubType();
         SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, variableChangeRequestPubSubType,
                                                                                       LogParticipantSettings.variableChangeTopic, ReliabilityKind.RELIABLE,
                                                                                       partition);
         subscriberAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
         domain.createSubscriber(participant, subscriberAttributes, new VariableChangeSubscriberListener());
      }
   }

   /**
    * Deactivate the data producer. 
    * 
    * After calling this function, the producer cannot be reactivated
    */
   public void remove()
   {
      domain.removeParticipant(participant);
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

   @Deprecated
   public int getPort()
   {
      return this.dataPort;
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

   private <T> Publisher createPersistentPublisher(String partition, String topicName, TopicDataType<T> topicDataType) throws IOException
   {
      PublisherAttributes publisherAttributes = domain.createPublisherAttributes(participant, topicDataType, topicName, ReliabilityKind.RELIABLE, partition);

      publisherAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
      publisherAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setKind(HistoryQosPolicyKind.KEEP_LAST_HISTORY_QOS);
      publisherAttributes.getTopic().getHistoryQos().setDepth(1);

      WriterTimes times = publisherAttributes.getTimes();
      times.getHeartbeatPeriod().setSeconds(0);
      times.getHeartbeatPeriod().setFraction(4294967 * 100);

      if (publisherAttributes.getQos().getPublishMode() == PublishModeKind.ASYNCHRONOUS_PUBLISH_MODE)
      {
         publisherAttributes.getThroughputController().setBytesPerPeriod(65000);
         publisherAttributes.getThroughputController().setPeriodMillisecs(1);
      }

      PublisherAttributes att = publisherAttributes;
      return domain.createPublisher(participant, att);

   }

   /** 
    * Add cameras to log 
    * 
    * Optional
    * 
    * @param name User friendly name to show in the log files
    * @param cameraId ID of the camera on the logger machine
    */
   public void addCamera(CameraType type, String name, String cameraId)
   {
      CameraAnnouncement cameraAnnouncement = new CameraAnnouncement();
      cameraAnnouncement.setType(type);
      cameraAnnouncement.setName(name);
      cameraAnnouncement.setIdentifier(cameraId);
      cameras.add(cameraAnnouncement);
   }

   /**
    * Activate the producer. This will publish the model, handshake and logger announcement to the logger dds network
    * 
    * @throws IOException
    */
   public void announce() throws IOException
   {
      if (activated)
      {
         throw new IOException("This participant is already activated.");
      }
      if (dataAddress == null || dataPort < 1024)
      {
         throw new RuntimeException("No data address and valid port (>=1024) provided");
      }
      if (handshake == null)
      {
         throw new RuntimeException("No handshake provided");
      }

      announcement.setHostName(InetAddress.getLocalHost().getHostName());
      announcement.setIdentifier(guidString);
      System.arraycopy(dataAddress.getAddress(), 0, announcement.getDataIP(), 0, 4);
      announcement.setDataPort((short) dataPort);
      announcement.setLog(log);

      for (CameraAnnouncement camera : cameras)
      {
         announcement.getCameras().add().set(camera);
      }

      handshakePublisher.write(handshake);
      announcementPublisher.write(announcement);
      
      activated = true;

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

   /** 
    * Publisher a timestamp update
    * @param timestamp
    * @throws IOException 
    */
   public void publishTimestamp(long timestamp) throws IOException
   {
      this.timestamp.setTimestamp(timestamp);
      timestampPublisher.write(this.timestamp);
   }

   public static void main(String[] args) throws IOException
   {
      InputStream testStream = DataProducerParticipant.class.getClassLoader().getResourceAsStream("Models/unitBox.sdf");
      String[] resourcesDirectories = {"Models"};
      LogModelProvider logModelProvider = new SDFLogModelProvider("testModel", testStream, resourcesDirectories);
      DataProducerParticipant participant = new DataProducerParticipant("testParticipant", logModelProvider, new VariableChangedListener()
      {

         @Override
         public void changeVariable(int id, double newValue)
         {
            System.out.println("Required change for " + id + " to " + newValue);
         }
      });

      participant.setDataAddress(Inet4Address.getLocalHost());
      participant.setPort(2048);
      participant.setHandshake(new Handshake());

      participant.announce();

      ThreadTools.sleepForever();
   }

}
