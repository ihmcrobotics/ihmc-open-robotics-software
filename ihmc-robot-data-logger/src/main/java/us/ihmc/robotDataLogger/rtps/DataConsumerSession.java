package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;

import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.attributes.DurabilityKind;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.ClearLogRequest;
import us.ihmc.robotDataLogger.ClearLogRequestPubSubType;
import us.ihmc.robotDataLogger.Timestamp;
import us.ihmc.robotDataLogger.TimestampPubSubType;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.listeners.ClearLogListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;

/**
 * A single connection to a variable server.
 * 
 * This session can be removed and a new one can be attached to allow re-connecting to a server
 * 
 * @author jesper
 *
 */
public class DataConsumerSession
{
   private final Announcement announcement;
   private final Domain domain;
   private final Participant participant;
   
   private final VariableChangedProducer variableChangedProducer;
   private final Publisher variableChangeDataPublisher;

   private final ClearLogRequest clearLogRequest = new ClearLogRequest();
   private final Publisher clearLogPublisher;
   

   private final RegistryConsumer registryConsumer;
   


   
   void remove()
   {
      if(variableChangeDataPublisher != null)
      {
         variableChangedProducer.setSession(null);
      }
      
      domain.removeParticipant(participant);
      registryConsumer.stopImmediatly();
      
   }
   
   DataConsumerSession(Domain domain, Announcement announcement, IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient, VariableChangedProducer variableChangedProducer, TimestampListener timeStampListener, ClearLogListener clearLogListener, RTPSDebugRegistry rtpsDebugRegistry) throws IOException
   {
      this.announcement = announcement;
      this.domain = domain;
      ParticipantAttributes att = domain.createParticipantAttributes(LogParticipantSettings.domain, "DataConsumerSession");
      this.participant = domain.createParticipant(att);
      
      this.variableChangedProducer = variableChangedProducer;
      if(variableChangedProducer != null)
      {
         VariableChangeRequestPubSubType topicDataType = new VariableChangeRequestPubSubType();
         PublisherAttributes attributes = domain.createPublisherAttributes(participant, topicDataType, LogParticipantSettings.variableChangeTopic, ReliabilityKind.RELIABLE, DataConsumerParticipant.getPartition(announcement.getIdentifierAsString()));
         variableChangeDataPublisher = domain.createPublisher(participant, attributes);
         variableChangedProducer.setSession(this);
      }
      else
      {
         variableChangeDataPublisher = null;
      }
      
      if(clearLogListener != null)
      {
         ClearLogRequestPubSubType clearLogRequestPubSubType = new ClearLogRequestPubSubType();
         PublisherAttributes publisherAttributes = domain.createPublisherAttributes(participant, clearLogRequestPubSubType, LogParticipantSettings.clearLogTopic, ReliabilityKind.RELIABLE, DataConsumerParticipant.getPartition(announcement.getIdentifierAsString()));
         publisherAttributes.getQos().setDurabilityKind(DurabilityKind.VOLATILE_DURABILITY_QOS); // make sure we do not persist
         clearLogRequest.setGuid(announcement.getIdentifierAsString());
         clearLogPublisher = domain.createPublisher(participant, publisherAttributes);
         
         SubscriberAttributes subscriberAttributes = domain.createSubscriberAttributes(participant, clearLogRequestPubSubType, LogParticipantSettings.clearLogTopic, ReliabilityKind.RELIABLE, DataConsumerParticipant.getPartition(announcement.getIdentifierAsString()));
         domain.createSubscriber(participant, subscriberAttributes, new ClearLogListenerImpl(clearLogListener, announcement.getIdentifierAsString()));
      }
      else
      {
         clearLogPublisher = null;
      }
      
      if(timeStampListener != null)
      {
         TimestampPubSubType pubSubType = new TimestampPubSubType();
         SubscriberAttributes attributes = domain.createSubscriberAttributes(participant, pubSubType, LogParticipantSettings.timestampTopic, ReliabilityKind.BEST_EFFORT, DataConsumerParticipant.getPartition(announcement.getIdentifierAsString()));
         domain.createSubscriber(participant, attributes, new TimestampListenerImpl(timeStampListener));

      }
      
      CustomLogDataSubscriberType pubSubType = new CustomLogDataSubscriberType(parser.getNumberOfVariables(), parser.getNumberOfJointStateVariables());
      SubscriberAttributes attributes = domain.createSubscriberAttributes(participant, pubSubType, LogParticipantSettings.dataTopic, ReliabilityKind.BEST_EFFORT, DataConsumerParticipant.getPartition(announcement.getIdentifierAsString()));
      registryConsumer = new RegistryConsumer(parser, yoVariableClient,rtpsDebugRegistry);
      domain.createSubscriber(participant, attributes, registryConsumer);
   }
   
   /**
    * Send a clear log request
    * 
    * @param announcement announcement of this domain as safe-guard
    * @throws IOException
    */
   public void sendClearLogRequest() throws IOException
   {
      if (clearLogPublisher == null)
      {
         return;
      }
      clearLogPublisher.write(clearLogRequest);
   }
   
   
   /**
    * Send a request to change variables
    * 
    * @param variableID
    * @param requestedValue
    * @throws IOException
    */
   void writeVariableChangeRequest(int variableID, double requestedValue) throws IOException
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
            if (subscriber.takeNextData(timestamp, info))
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

      }

   }

   Announcement getAnnouncement()
   {
      return announcement;
   }

   
   
   
   
}
