package us.ihmc.avatar.ros.visualizer;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.common.DiscoveryStatus;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.participant.Participant;

import java.util.concurrent.atomic.AtomicLong;

public class ROS2Debugger
{
   private Participant participant;
   private AtomicLong numberOfParticipants = new AtomicLong();
   private AtomicLong numberOfPublishers = new AtomicLong();
   private AtomicLong numberOfSubscribers = new AtomicLong();
   private AtomicLong numberOfEndpoints = new AtomicLong();
   private AtomicLong numberOfMatches = new AtomicLong();

   public ROS2Debugger()
   {
      ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   private void setup() throws Exception
   {
      int domainID = NetworkParameters.getRTPSDomainID();
      Domain domain = DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS);
      ParticipantAttributes attributes = domain.createParticipantAttributes();
      attributes.setDomainId(domainID);
      attributes.setLeaseDuration(Time.Infinite);
      attributes.setName(getClass().getSimpleName());

      participant = domain.createParticipant(attributes, (participant, info) ->
      {
         if (!info.getGuid().equals(this.participant.getGuid()))
         {
            if (info.getStatus() == DiscoveryStatus.DISCOVERED_RTPSPARTICIPANT)
            {
               numberOfParticipants.incrementAndGet();
               LogTools.info("Discovered participant: {}", info.getName());
            }
            else if (info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
            {
               numberOfParticipants.decrementAndGet();
               LogTools.info("Participant removed: {}", info.getName());
            }
         }
      });
      participant.registerEndpointDiscoveryListeners(
      ((isAlive, guid, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, typeMaxSerialized, topicKind, writerQosHolder) ->
      {
         numberOfPublishers.incrementAndGet();
         numberOfEndpoints.incrementAndGet();
         LogTools.info("Discovered publisher on topic: {}", topicName);
      }),
      ((isAlive, guid, expectsInlineQos, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, javaTopicKind, readerQosHolder) ->
      {
         numberOfSubscribers.incrementAndGet();
         numberOfEndpoints.incrementAndGet();
         LogTools.info("Discovered subscriber on topic: {}", topicName);
      }));
   }

   public long getNumberOfParticipants()
   {
      return numberOfParticipants.get();
   }

   public long getNumberOfPublishers()
   {
      return numberOfPublishers.get();
   }

   public long getNumberOfSubscribers()
   {
      return numberOfSubscribers.get();
   }

   public long getNumberOfEndpoints()
   {
      return numberOfEndpoints.get();
   }

   public static void main(String[] args)
   {
      new ROS2Debugger();
      ThreadTools.sleepForever();
   }
}
