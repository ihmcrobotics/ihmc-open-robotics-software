package us.ihmc.robotDataLogger.guiRecorder;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.pubsub.types.ByteBufferPubSubType;

public class GUICaptureReceiver implements SubscriberListener
{
   private final String topicName;
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private final GUICaptureHandler handler;
   private final ByteBuffer receiveBuffer = ByteBuffer.allocateDirect(GUICaptureStreamer.MAXIMUM_IMAGE_DATA_SIZE);
   
   
   public GUICaptureReceiver(int domainID, String topicName, GUICaptureHandler handler) throws IOException
   {
      this.topicName = topicName;
      this.handler = handler;

      ParticipantAttributes participantAttributes = domain.createParticipantAttributes(domainID, getClass().getSimpleName());
      participant = domain.createParticipant(participantAttributes);

   }

   public void start() throws IllegalArgumentException, IOException
   {
      ByteBufferPubSubType pubSubType = new ByteBufferPubSubType(GUICaptureStreamer.topicType, GUICaptureStreamer.MAXIMUM_IMAGE_DATA_SIZE);
      SubscriberAttributes attributes = domain.createSubscriberAttributes(participant, pubSubType, topicName, ReliabilityKind.BEST_EFFORT, GUICaptureStreamer.partition);
      domain.createSubscriber(participant, attributes, this);

   }
  
   
   public void close()
   {
      domain.removeParticipant(participant);
   }

   @Override
   public void onNewDataMessage(Subscriber subscriber)
   {
      try
      {
         receiveBuffer.clear();
         if(subscriber.takeNextData(receiveBuffer, null))
         {
            handler.receivedFrame(receiveBuffer);
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