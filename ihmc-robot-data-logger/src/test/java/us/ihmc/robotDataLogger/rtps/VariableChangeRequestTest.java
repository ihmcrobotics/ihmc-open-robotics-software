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
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.common.LogLevel;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.MatchingInfo.MatchingStatus;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.pubsub.subscriber.SubscriberListener;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;

public class VariableChangeRequestTest
{

   @ContinuousIntegrationTest(estimatedDuration = 11)
   @Test(timeout = 100000)
   public void testSendingVariableChangedMessage() throws IOException
   {
      Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
      domain.setLogLevel(LogLevel.WARNING);

      ParticipantAttributes attr = domain.createParticipantAttributes(1, "TestParticipant");
      attr.bindToLocalhost();
      Participant participant = domain.createParticipant(attr);

      VariableChangeRequestPubSubType type = new VariableChangeRequestPubSubType();

      Publisher publisher1 = domain.createPublisher(participant,
                                                    domain.createPublisherAttributes(participant, type, "testTopic", ReliabilityKind.RELIABLE, "us/ihmc"));
      Publisher publisher2 = domain.createPublisher(participant,
                                                    domain.createPublisherAttributes(participant, type, "testTopic", ReliabilityKind.RELIABLE, "us/ihmc"));

      AtomicInteger receivedMessages = new AtomicInteger(0);

      SubscriberListener listener = new SubscriberListener()
      {

         @Override
         public void onSubscriptionMatched(Subscriber subscriber, MatchingInfo info)
         {
            if(info.getStatus() == MatchingStatus.MATCHED_MATCHING)
            {
               System.out.println("Connected " + info.getGuid());
            }
            else
            {
               System.out.println("Disconnected " + info.getGuid());
            }
         }

         @Override
         public void onNewDataMessage(Subscriber subscriber)
         {
            VariableChangeRequest req = new VariableChangeRequest();
            SampleInfo info = new SampleInfo();
            if (subscriber.takeNextData(req, info))
            {
               receivedMessages.incrementAndGet();
            }
            else
            {
               System.err.println("Could not decode message");
            }
         }
      };

      domain.createSubscriber(participant, domain.createSubscriberAttributes(participant, type, "testTopic", ReliabilityKind.RELIABLE, "us/ihmc"), listener);

      ThreadTools.sleep(1000);

      for (int i = 0; i < 10; i++)
      {
         VariableChangeRequest msg = new VariableChangeRequest();
         msg.setVariableID(i + 100);
         msg.setRequestedValue(i * 13.37);

         publisher1.write(msg);

         VariableChangeRequest msg2 = new VariableChangeRequest();
         msg2.setVariableID(i + 200);
         msg2.setRequestedValue(i * 2.0);

         publisher2.write(msg2);
         ThreadTools.sleep(100);
      }

      assertEquals(20, receivedMessages.get());
   }

}
