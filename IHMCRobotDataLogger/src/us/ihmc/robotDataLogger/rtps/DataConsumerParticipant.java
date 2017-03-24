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
import us.ihmc.robotDataLogger.ClearLogRequest;
import us.ihmc.robotDataLogger.ClearLogRequestPubSubType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.Timestamp;
import us.ihmc.robotDataLogger.TimestampPubSubType;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.robotDataLogger.listeners.ClearLogListener;
import us.ihmc.robotDataLogger.listeners.LogAnnouncementListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;

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
   
   private Publisher clearLogPublisher = null;

   private class TimestampListenerImpl implements SubscriberListener
   {
      private final Timestamp timestamp = new Timestamp();
      private final SampleInfo info = new SampleInfo();
      private final TimestampListener listener;
      
      
      
      public TimestampListenerImpl(TimestampListener listener)
      {
         this.listener = listener;
      }

      @Override
      public void onNewDataMessage(Subscriber subscriber)
      {
         try
         {
            if(subscriber.takeNextData(timestamp, info))
            {
               listener.receivedTimestampOnly(timestamp.getTimestamp());
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
         // TODO Auto-generated method stub
         
      }
      
      
   }
   
   private class ClearLogListenerImpl implements SubscriberListener
   {
      private final String logGuid;
      private final ClearLogListener clearLogListener;
      
      private ClearLogListenerImpl(ClearLogListener clearLogListener, String logGuid)
      {
         this.clearLogListener = clearLogListener;
         this.logGuid = logGuid;
      }
         
      @Override
      public void onNewDataMessage(Subscriber subscriber)
      {
         ClearLogRequest clearLogRequest = new ClearLogRequest();
         SampleInfo info = new SampleInfo();

         try
         {
            if (subscriber.takeNextData(clearLogRequest, info))
            {
               if (clearLogListener != null && clearLogRequest.getGuidAsString().equals(logGuid))
               {
                  clearLogListener.clearLog(LogParticipantTools.createGuidString(info.getSampleIdentity().getGuid()));
               }
               else
               {
                  System.err.println("Clear log guid is invalid");
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
         // TODO Auto-generated method stub
         
      }
      
   }
   
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
      domain.setLogLevel(LogLevel.ERROR);
      ParticipantAttributes att = domain.createParticipantAttributes(LogParticipantSettings.domain, name);
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
      SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, announcementPubSubType, LogParticipantSettings.annoucementTopic, ReliabilityKind.RELIABLE, LogParticipantSettings.partition);
      subscriberAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      domain.createSubscriber(participant, subscriberAttributes, new AnnouncementListener());

   }

   private <T> T getData(T data, TopicDataType<T> topicDataType, Announcement announcement, String topic, int timeout) throws IOException
   {
      SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, topicDataType, topic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      subscriberAttributes.getQos().setReliabilityKind(ReliabilityKind.RELIABLE);
      subscriberAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      Subscriber subscriber = domain.createSubscriber(participant, subscriberAttributes);
      try
      {
         subscriber.waitForUnreadMessage(timeout);
      }
      catch (InterruptedException e)
      {
         domain.removeSubscriber(subscriber);
         throw new IOException("Did not receive data from " + topic + " within " + timeout + " milliseconds");
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
   public byte[] getModelFile(Announcement announcement, int timeout) throws IOException
   {
      if(!announcement.getModelFileDescription().getHasModel())
      {
         throw new RuntimeException("This session does not have a model");
      }
      
      byte[] data = new byte[announcement.getModelFileDescription().getModelFileSize()];
      ByteBufferPubSubType byteBufferPubSubType = new ByteBufferPubSubType(LogParticipantSettings.modelFileTypeName, data.length);
      
      getData(ByteBuffer.wrap(data), byteBufferPubSubType, announcement, LogParticipantSettings.modelFileTopic, timeout);
      
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
   public byte[] getResourceZip(Announcement announcement, int timeout) throws IOException
   {
      if(!announcement.getModelFileDescription().getHasResourceZip())
      {
         throw new RuntimeException("This session does not have a resource bundle");
      }
      
      byte[] data = new byte[announcement.getModelFileDescription().getResourceZipSize()];
      ByteBufferPubSubType byteBufferPubSubType = new ByteBufferPubSubType(LogParticipantSettings.resourceBundleTypeName, data.length);
      
      getData(ByteBuffer.wrap(data), byteBufferPubSubType, announcement, LogParticipantSettings.resourceBundleTopic, timeout);
      
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
   public Handshake getHandshake(Announcement announcement, int timeout) throws IOException
   {
      HandshakePubSubType handshakePubSubType = new HandshakePubSubType();
      return getData(new Handshake(), handshakePubSubType, announcement, LogParticipantSettings.handshakeTopic, timeout);
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
      PublisherAttributes attributes = domain.createPublisherAttributes(participant, topicDataType, LogParticipantSettings.variableChangeTopic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      variableChangeDataPublisher = domain.createPublisher(participant, attributes);
   }
   
   /**
    * Send a request to change variables
    * 
    * @param variableID
    * @param requestedValue
    * @throws IOException
    */
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

   
   /**
    * Create a clear log publisher/subscriber pair. Clear log gets send to all subscribers off the producer
    * 
    * @param listener
    * @throws IOException 
    */
   public void createClearLogPubSub(Announcement announcement, ClearLogListener listener) throws IOException
   {
      ClearLogRequestPubSubType clearLogRequestPubSubType = new ClearLogRequestPubSubType();
      PublisherAttributes publisherAttributes = domain.createPublisherAttributes(participant, clearLogRequestPubSubType, LogParticipantSettings.clearLogTopic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      publisherAttributes.getQos().setDurabilityKind(DurabilityKind.VOLATILE_DURABILITY_QOS); // make sure we do not persist
      clearLogPublisher = domain.createPublisher(participant, publisherAttributes);
      
      SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, clearLogRequestPubSubType, LogParticipantSettings.clearLogTopic, ReliabilityKind.RELIABLE, getPartition(announcement.getIdentifierAsString()));
      domain.createSubscriber(participant, subscriberAttributes, new ClearLogListenerImpl(listener, announcement.getIdentifierAsString()));
   }
   
   /**
    * Send a clear log request
    * 
    * @param announcement announcement of this domain as safe-guard
    * @throws IOException
    */
   public void sendClearLogRequest(Announcement announcement) throws IOException
   {
      ClearLogRequest request = new ClearLogRequest();
      request.setGuid(announcement.getIdentifierAsString());
      clearLogPublisher.write(request);
   }
   

   public void createTimestampListener(Announcement announcement, TimestampListener listener) throws IOException
   {
      TimestampPubSubType pubSubType = new TimestampPubSubType();
      SubscriberAttributes attributes = domain.createSubscriberAttributes(participant, pubSubType, LogParticipantSettings.timestampTopic, ReliabilityKind.BEST_EFFORT, getPartition(announcement.getIdentifierAsString()));
      domain.createSubscriber(participant, attributes, new TimestampListenerImpl(listener));
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
         System.out.println(dataConsumerParticipant.getHandshake(received, 15000));
         System.out.println(dataConsumerParticipant.getModelFile(received, 15000).length);
         System.out.println(dataConsumerParticipant.getResourceZip(received, 15000).length);
      }

   }

   public void remove()
   {
      domain.removeParticipant(participant);
   }

}
