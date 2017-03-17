package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.participant.Participant;

public class DataProducerParticipant
{
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private final Guid guid;
   
   public DataProducerParticipant(String name) throws IOException
   {
      ParticipantAttributes<?> att = domain.createParticipantAttributes();
      att.setDomainId(LogParticipantSettings.domain);
      att.setLeaseDuration(Time.Infinite);
      att.setName(name);
      participant = domain.createParticipant(att, null);
      guid = participant.getGuid();
      
   }
   
   public void setModelFileProvider(LogModelProvider provider)
   {
      
   }
  
   
}
