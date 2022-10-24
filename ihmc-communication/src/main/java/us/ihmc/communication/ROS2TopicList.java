package us.ihmc.communication;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.common.DiscoveryStatus;
import us.ihmc.pubsub.participant.Participant;

public class ROS2TopicList
{
   public static void main(String[] args) throws Exception
   {
      int domainID = NetworkParameters.getRTPSDomainID();
      Domain domain = DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS);
      ParticipantAttributes attributes = domain.createParticipantAttributes(domainID, ROS2TopicList.class.getSimpleName());

      Participant participant = domain.createParticipant(attributes, (participantLocal, info) ->
      {
         if (!info.getGuid().equals(participantLocal.getGuid()))
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
            ((isAlive, guid, participantGuid, typeName, topicName, userDefinedId, typeMaxSerialized, topicKind) ->
            {
               LogTools.info("Discovered publisher on topic: {}", topicName);
            }),
            ((isAlive, guid, expectsInlineQos, participantGuid, typeName, topicName, userDefinedId, javaTopicKind) ->
            {
               LogTools.info("Discovered subscriber on topic: {}", topicName);
            }));
      ThreadTools.sleepForever();
   }
}
