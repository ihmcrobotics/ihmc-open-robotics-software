package us.ihmc.communication;

import java.io.IOException;
import java.util.Random;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.participant.Participant;

/**
 * Creates and Manages participants
 */
public class RTPSCommunicationFactory
{
   private static int START_OF_RANDOM_DOMAIN_RANGE = 200;
   private final Domain domain = DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS);
   private final TIntObjectHashMap<Participant> participants = new TIntObjectHashMap<>();
   private final int defaultDomainID;
   
   /**
    * Creates an RTPSCommunicationFactory. Loads the default RTPS Domain ID from the Network Parameter File on disk.
    * This file is typically located in the user home directory /.ihmc/IHMCNetworkParameters.ini
    * If the domain ID is not found, a random ID is generated between 800 and 900
    */
   public RTPSCommunicationFactory()
   {
      int rtpsDomainID = new Random().nextInt(50) + START_OF_RANDOM_DOMAIN_RANGE;
      if (NetworkParameters.hasKey(NetworkParameterKeys.RTPSDomainID))
      {
         rtpsDomainID = NetworkParameters.getRTPSDomainID();
         PrintTools.info("Setting the RTPS Domain ID to " + rtpsDomainID);
      }
      else
      {
         PrintTools.error("No RTPS Domain ID set in the NetworkParameters file. The entry should look like RTPSDomainID:15, setting the Default RTPS Domain ID to " + rtpsDomainID);
      }
      
      defaultDomainID = rtpsDomainID;
      createParticipant(rtpsDomainID);
   }
   
   /**
    * Returns a handle on the singleton instance of the Domain
    * @return
    */
   public Domain getDomain()
   {
      return domain;
   }
   
   /**
    * Returns a participant attached to the domain. 
    */
   public Participant getOrCreateParticipant(int domainID)
   {
      if(!participants.containsKey(domainID))
      {
         createParticipant(domainID);
      }
      return participants.get(domainID);
   }
   
   /**
    * creates a participant using the default domain ID. The Default domain ID is either loaded from file or
    * randomly generated in the 800-900 range
    * @return the default participant
    */
   public Participant getDefaultParticipant()
   {
      return getOrCreateParticipant(defaultDomainID);
   }

   /**
    * Creates a participant attached to the domain using the specified domain ID
    * @param domainId the id to use for the domain
    * @return a new participant attached to the domain
    */
   private void createParticipant(int domainId)
   {
      ParticipantAttributes attributes = domain.createParticipantAttributes(domainId, RTPSCommunicationFactory.class.getSimpleName());
      try
      {
         participants.put(domainId, domain.createParticipant(attributes));
      }
      catch (IOException e)
      {
         System.err.println("Could not create pub sub participant.");
         throw new RuntimeException(e);
      }
   }
   
   /**
    * Quick test method to ensure the factory correctly loads the rtps domain ID.
    * @param args none needed
    */
   public static void main(String[] args)
   {
      new RTPSCommunicationFactory().getDefaultParticipant();
   }
}
