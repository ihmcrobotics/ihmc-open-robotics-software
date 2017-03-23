package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.DurabilityKind;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.pubsub.common.DiscoveryStatus;
import us.ihmc.pubsub.common.Guid.GuidPrefix;
import us.ihmc.pubsub.common.LogLevel;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.participant.ParticipantDiscoveryInfo;
import us.ihmc.pubsub.participant.ParticipantListener;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.AnnouncementPubSubType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;

/**
 * This class implements all communication for a data consumer inside a DDS logging network
 * 
 * @author jesper
 *
 */
public class DataConsumerParticipant
{

   private final ReentrantLock lock = new ReentrantLock();
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private LogAnnouncementListener logAnnouncementListener;
   private final HashMap<GuidPrefix, Announcement> announcements = new HashMap<>();
   private Publisher variableChangeDataPublisher = null;

   private class LeaveListener implements ParticipantListener
   {
      @Override
      public void onParticipantDiscovery(Participant participant, ParticipantDiscoveryInfo info)
      {
         if (info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
         {
            lock.lock();
            Announcement removed = announcements.remove(info.getGuid().getGuidPrefix());
            if (removed != null)
            {
               if (logAnnouncementListener != null)
               {
                  logAnnouncementListener.logSessionWentOffline(removed);
               }
            }
            lock.unlock();
         }
      }

   }

   private class AnnouncementListener implements SubscriberListener
   {

      @Override
      public void onNewDataMessage(Subscriber subscriber)
      {
         Announcement announcement = new Announcement();
         SampleInfo info = new SampleInfo();
         try
         {
            if (subscriber.takeNextData(announcement, info))
            {
               lock.lock();
               GuidPrefix guid = info.getSampleIdentity().getGuid().getGuidPrefix();
               if (announcements.containsKey(guid))
               {
                  // Ignore duplicate announcements
               }
               else
               {
                  announcements.put(guid, announcement);
                  if (logAnnouncementListener != null)
                  {
                     logAnnouncementListener.logSessionCameOnline(announcement);
                  }
               }
               lock.unlock();
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

   /**
    * Constructor 
    * 
    * @param name for this log consumer participant
    * @throws IOException if no connection to the network is possibile
    */
   public DataConsumerParticipant(String name) throws IOException
   {
      domain.setLogLevel(LogLevel.WARNING);
      ParticipantAttributes att = domain.createDefaultParticipantAttributes(LogParticipantSettings.domain, name);
      participant = domain.createParticipant(att, new LeaveListener());
   }

   private String getPartition(String guid)
   {
      return LogParticipantSettings.partition + LogParticipantSettings.namespaceSeperator + guid;
   }

   /**
    * Subscribe to the announce topic and callback on new sessions
    * 
    * @param listener Callback listener
    * @throws IOException 
    */
   public void listenForAnnouncements(LogAnnouncementListener listener) throws IOException
   {
      if (listener == null)
      {
         throw new RuntimeException("Listener is null");
      }

      this.logAnnouncementListener = listener;

      AnnouncementPubSubType announcementPubSubType = new AnnouncementPubSubType();
      SubscriberAttributes subscriberAttributes = domain.createDefaultSubscriberAttributes(participant, announcementPubSubType, LogParticipantSettings.annoucementTopic, ReliabilityKind.RELIABLE, LogParticipantSettings.partition);
      subscriberAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      domain.createSubscriber(participant, subscriberAttributes, new AnnouncementListener());

   }

   private <T> T getData(T data, TopicDataType<T> topicDataType, Announcement announcement, String topic) throws IOException
   {
      SubscriberAttributes subscriberAttributes = domain.createDefaultSubscriberAttributes(participant, topicDataType, topic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      subscriberAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
      subscriberAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      Subscriber subscriber = domain.createSubscriber(participant, subscriberAttributes, null);
      try
      {
         subscriber.waitForUnreadMessage(LogParticipantSettings.handshakeTimeout);
      }
      catch (InterruptedException e)
      {
         domain.removeSubscriber(subscriber);
         throw new IOException("Did not receive data from " + topic + " within " + LogParticipantSettings.handshakeTimeout + " milliseconds");
      }

      SampleInfo sampleInfo = new SampleInfo();
      if (subscriber.takeNextData(data, sampleInfo))
      {
         domain.removeSubscriber(subscriber);
         return data;
      }
      else
      {
         domain.removeSubscriber(subscriber);
         throw new IOException("Did not receive data from " + topic);
      }

   }

   /**
    * Requests the model file 
    *  
    * @param announcement
    * 
    * @return byte[] array of the model file
    * 
    * @throws IOException if no reply has been received within  the timeout
    * @throws RuntimeException if no model file is announced in the announcement
    */
   public byte[] getModelFile(Announcement announcement) throws IOException
   {
      if(!announcement.getModelFileDescription().getHasModel())
      {
         throw new RuntimeException("This session does not have a model");
      }
      
      byte[] data = new byte[announcement.getModelFileDescription().getModelFileSize()];
      ByteBufferPubSubType byteBufferPubSubType = new ByteBufferPubSubType(LogParticipantSettings.modelFileTypeName, data.length);
      
      getData(ByteBuffer.wrap(data), byteBufferPubSubType, announcement, LogParticipantSettings.modelFileTopic);
      
      return data;
      
   }
   /**
    * Requests the resource zip 
    *  
    * @param announcement
    * 
    * @return byte[] array of the resource bundle
    * 
    * @throws IOException if no reply has been received within the timeout
    * @throws RuntimeException if no resource bundle is announced in the announcement
    */
   public byte[] getResourceZip(Announcement announcement) throws IOException
   {
      if(!announcement.getModelFileDescription().getHasResourceZip())
      {
         throw new RuntimeException("This session does not have a resource bundle");
      }
      
      byte[] data = new byte[announcement.getModelFileDescription().getResourceZipSize()];
      ByteBufferPubSubType byteBufferPubSubType = new ByteBufferPubSubType(LogParticipantSettings.resourceBundleTypeName, data.length);
      
      getData(ByteBuffer.wrap(data), byteBufferPubSubType, announcement, LogParticipantSettings.resourceBundleTopic);
      
      return data;
      
   }


   /**
    * Request the handshake 
    * 
    * 
    * @param announcement 
    * 
    * @return Handshake
    * 
    * @throws IOException if no reply has been received within the timeout
    */
   public Handshake getHandshake(Announcement announcement) throws IOException
   {
      HandshakePubSubType handshakePubSubType = new HandshakePubSubType();
      return getData(new Handshake(), handshakePubSubType, announcement, LogParticipantSettings.handshakeTopic);
   }
   
   /**
    * Create a variable change producer 
    * 
    * @param announcement
    * @throws IOException if the variable change producer is already created or cannot be created
    */
   public void createVariableChangeProducer(Announcement announcement) throws IOException
   {
      if(variableChangeDataPublisher != null)
      {
         throw new RuntimeException("Variable change producer is already created");
      }

      VariableChangeRequestPubSubType topicDataType = new VariableChangeRequestPubSubType();
      PublisherAttributes attributes = domain.createDefaultPublisherAttributes(participant, topicDataType, LogParticipantSettings.variableChangeTopic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      variableChangeDataPublisher = domain.createPublisher(participant, attributes, null);
   }
   
   
   public void writeVariableChangeRequest(int variableID, double requestedValue) throws IOException
   {
      if(variableChangeDataPublisher == null)
      {
         throw new RuntimeException("No variable change data publisher created");
      }
      
      VariableChangeRequest request = new VariableChangeRequest();
      request.setVariableID(variableID);
      request.setRequestedValue(requestedValue);
      variableChangeDataPublisher.write(request);
   }

   

   public static void main(String[] args) throws IOException, InterruptedException
   {
      DataConsumerParticipant dataConsumerParticipant = new DataConsumerParticipant("testConsumer");

      ArrayBlockingQueue<Announcement> queue = new ArrayBlockingQueue<>(1);
      dataConsumerParticipant.listenForAnnouncements(new LogAnnouncementListener()
      {

         @Override
         public void logSessionWentOffline(Announcement announcement)
         {
            System.out.println(announcement + " went offline");
         }

         @Override
         public void logSessionCameOnline(Announcement announcement)
         {
            System.out.println(announcement + " came online");
            try
            {
               queue.put(announcement);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      });

      Announcement received;
      if((received = queue.take()) != null)
      {
         System.out.println(dataConsumerParticipant.getHandshake(received));
         System.out.println(dataConsumerParticipant.getModelFile(received).length);
         System.out.println(dataConsumerParticipant.getResourceZip(received).length);
      }

   }

}
