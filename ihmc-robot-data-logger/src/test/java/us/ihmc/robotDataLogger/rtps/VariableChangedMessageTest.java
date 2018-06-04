package us.ihmc.robotDataLogger.rtps;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;

public class VariableChangedMessageTest
{
      
   
   
      @ContinuousIntegrationTest(estimatedDuration = 11)
      @Test(timeout = 100000)
      public void testSendingVariableChangedMessage() throws IOException
      {
         Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
         
         
         Participant participant = domain.createParticipant(domain.createParticipantAttributes(1, "TestParticipant"));
         
         
         VariableChangeRequestPubSubType type = new VariableChangeRequestPubSubType();
         
         Publisher publisher = domain.createPublisher(participant, domain.createPublisherAttributes(participant, type, "testTopic", ReliabilityKind.RELIABLE, "us.ihmc"));
         
         
         AtomicInteger receivedMessages = new AtomicInteger(0);
         
         SubscriberListener listener = new SubscriberListener()
         {
            
            @Override
            public void onSubscriptionMatched(Subscriber subscriber, MatchingInfo info)
            {
               // TODO Auto-generated method stub
               
            }
            
            @Override
            public void onNewDataMessage(Subscriber subscriber)
            {
               VariableChangeRequest req = new VariableChangeRequest();
               if(subscriber.takeNextData(req, new SampleInfo()))
               {
                  receivedMessages.incrementAndGet();
               }
               else
               {
                  System.err.println("COULD NOT DECODE MESSAGE");
               }
            }
         };
         
         domain.createSubscriber(participant, domain.createSubscriberAttributes(participant, type, "testTopic", ReliabilityKind.RELIABLE, "us.ihmc"), listener);
         
         ThreadTools.sleep(1000);
         
         for(int i = 0;  i < 10; i++)
         {
            VariableChangeRequest msg = new VariableChangeRequest();
            msg.variableID_ = i + 100;
            msg.requestedValue_ = i * 13.37;
            
            publisher.write(msg);
            ThreadTools.sleep(1000);
         }
         
         
         assertEquals(10, receivedMessages.get());
      }
   
}  
