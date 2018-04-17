package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.LinkedHashMap;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.DurabilityKind;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
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
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.AnnouncementPubSubType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
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

   private final ReentrantLock announcementLock = new ReentrantLock();
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private Participant participant;
   private LogAnnouncementListener logAnnouncementListener;
   private final LinkedHashMap<GuidPrefix, Announcement> announcements = new LinkedHashMap<>();

   private DataConsumerSession session;

   private class LeaveListener implements ParticipantListener
   {
      @Override
      public void onParticipantDiscovery(Participant participant, ParticipantDiscoveryInfo info)
      {
         if (info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
         {
            announcementLock.lock();
            Announcement removed = announcements.remove(info.getGuid().getGuidPrefix());
            if (removed != null)
            {
               if (logAnnouncementListener != null)
               {
                  logAnnouncementListener.logSessionWentOffline(removed);
               }
               disconnectIfActiveSession(removed);
            }
            announcementLock.unlock();
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
               announcementLock.lock();
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
               announcementLock.unlock();
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
    * Get the first matching announcement that is compatible with the original announcement.
    * 
    * The announcement returned is the latest added, such that the newest session is connected to.
    * 
    * @param originalAnnouncement
    */
   public Announcement getReconnectableSession(Announcement originalAnnouncement)
   {
      Announcement newAnnouncement = null;
      
      announcementLock.lock();
      for(Announcement announcement : announcements.values())
      {
         System.out.println("Trying announcement " + announcement);
         
         
         if(announcement.getHostNameAsString().equals(originalAnnouncement.getHostNameAsString())
               && announcement.getReconnectKeyAsString().equals(originalAnnouncement.getReconnectKeyAsString()) 
               && announcement.getNameAsString().equals(originalAnnouncement.getNameAsString()))
         {
            newAnnouncement = announcement;
         }
      }
      announcementLock.unlock();
      
      return newAnnouncement;
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

   static String getPartition(String guid)
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
      SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, announcementPubSubType,
                                                                                    LogParticipantSettings.annoucementTopic, ReliabilityKind.RELIABLE,
                                                                                    LogParticipantSettings.partition);
      subscriberAttributes.getQos().setDurabilityKind(DurabilityKind.TRANSIENT_LOCAL_DURABILITY_QOS);
      domain.createSubscriber(participant, subscriberAttributes, new AnnouncementListener());

   }

   private <T> T getData(T data, TopicDataType<T> topicDataType, Announcement announcement, String topic, int timeout) throws IOException
   {
      SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, topicDataType, topic, ReliabilityKind.RELIABLE,
                                                                                    getPartition(announcement.getIdentifierAsString()));
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
      if (!announcement.getModelFileDescription().getHasModel())
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
      if (!announcement.getModelFileDescription().getHasResourceZip())
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
    * Create a new variable receiver session
    * 
    * Will throw an error if the session is already running or if the participant has been removed from the domain.
    * 
    * @param announcement
    * @param parser
    * @param yoVariableClient
    * @param variableChangedProducer
    * @param timeStampListener
    * @param clearLogListener
    * @param rtpsDebugRegistry
    * @return
    * @throws IOException
    */
   public synchronized DataConsumerSession createSession(Announcement announcement, IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient,
                                                         VariableChangedProducer variableChangedProducer, TimestampListener timeStampListener,
                                                         ClearLogListener clearLogListener, RTPSDebugRegistry rtpsDebugRegistry)
         throws IOException
   {
      if (session != null)
      {
         throw new IOException("Session is already connected");
      }

      if(participant == null)
      {
         throw new IOException("Participant has been removed from the domain. Cannot create a new session");
      }
      
      session = new DataConsumerSession(domain, announcement, parser, yoVariableClient, variableChangedProducer, timeStampListener,
                                        clearLogListener, rtpsDebugRegistry);
      return session;
   }

   /**
    * Disconnect the current sessions, if connected.
    * 
    * After disconnecting the sessions it is still possible to reconnect to another session.
    * 
    */
   public synchronized void disconnectSession()
   {
      if (session != null)
      {
         session.remove();
         session = null;
      }
   }

   /**
    * Remove the participant from the domain.
    * 
    * After calling this function 
    */
   public synchronized void remove()
   {
      if (participant != null)
      {
         disconnectSession();
         domain.removeParticipant(participant);
         participant = null;

      }
   }

   /**
    * 
    * @return true if the participant is connected to the domain
    */
   public synchronized boolean isConnectedToDomain()
   {
      return participant != null;
   }
   
   /**
    * 
    * @return true if a session is currently active
    */
   public synchronized boolean isSessionActive()
   {
      return session != null;
   }
   
   
   
   /**
    * Broadcast a clear log request for the current session
    * 
    * If no session is available, this request gets silently ignored.
    * 
    * @throws IOException
    */
   public synchronized void sendClearLogRequest() throws IOException
   {
      if (session != null)
      {
         session.sendClearLogRequest();
      }
   }

   
   synchronized void disconnectIfActiveSession(Announcement announcement)
   {
      if(session != null)
      {
         if(session.getAnnouncement().equals(announcement))
         {
            disconnectSession();
         }
      }
   }
}
