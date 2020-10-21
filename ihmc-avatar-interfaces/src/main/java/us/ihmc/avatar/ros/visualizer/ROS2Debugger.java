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

public class ROS2Debugger
{
   private Participant participant;

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
               LogTools.info("Discovered participant: {}", info.getName());
            }
            else if (info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
            {
               LogTools.info("Participant removed: {}", info.getName());
            }
         }
      });
      participant.registerEndpointDiscoveryListeners(
      ((isAlive, guid, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, typeMaxSerialized, topicKind, writerQosHolder) ->
      {
         LogTools.info("Discovered publisher on topic: {}", topicName);
      }),
      ((isAlive, guid, expectsInlineQos, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, javaTopicKind, readerQosHolder) ->
      {
         LogTools.info("Discovered subscriber on topic: {}", topicName);
      }));

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ROS2Debugger();
   }
}
