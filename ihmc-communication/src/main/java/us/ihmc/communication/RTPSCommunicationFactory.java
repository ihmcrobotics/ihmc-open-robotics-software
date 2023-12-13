package us.ihmc.communication;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.net.util.SubnetUtils;

import gnu.trove.map.hash.TIntObjectHashMap;
import org.apache.commons.net.util.SubnetUtils.SubnetInfo;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.participant.Participant;

/**
 * Creates and Manages participants
 */
public class RTPSCommunicationFactory
{
   private static final int START_OF_RANDOM_DOMAIN_RANGE = 200;
   private static final List<InterfaceAddress> MACHINE_INTERFACE_ADDRESSES = findInterfaceAddresses();

   private final Domain domain = DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS);
   private final TIntObjectHashMap<Participant> participants = new TIntObjectHashMap<>();
   private final int defaultDomainID;
   private final InetAddress defaultAddressRestriction;

   /**
    * Creates an RTPSCommunicationFactory. Loads the default RTPS Domain ID from the Network Parameter
    * File on disk. This file is typically located in the user home directory
    * /.ihmc/IHMCNetworkParameters.ini If the domain ID is not found, a random ID is generated between
    * 200 and 229
    */
   public RTPSCommunicationFactory()
   {
      int rtpsDomainID = new Random().nextInt(30) + START_OF_RANDOM_DOMAIN_RANGE;

      if (NetworkParameters.hasKey(NetworkParameterKeys.RTPSDomainID))
      {
         rtpsDomainID = NetworkParameters.getRTPSDomainID();
         LogTools.info("Using DDS/ROS 2 Domain ID {}", rtpsDomainID);
      }
      else
      {
         LogTools.error("""
                     Tried to load the RTPS Domain ID from %s, \
                     but either the key didn't exist or after parsing the string it \
                     returned a negative number.
                     It should look like RTPSDomainID=15, if your registered domain ID is 15.
                     Setting the Default RTPS Domain ID randomly to %s, to avoid interfering with others.\
                     """.formatted(NetworkParameters.defaultParameterFile, rtpsDomainID));
      }

      InetAddress foundAddressRestriction = null;

      String restrictionHost;
      if (MACHINE_INTERFACE_ADDRESSES != null
       && NetworkParameters.hasKey(NetworkParameterKeys.RTPSSubnet)
       && !(restrictionHost = NetworkParameters.getHost(NetworkParameterKeys.RTPSSubnet)).isEmpty())
      {
         if (SystemUtils.IS_OS_WINDOWS && !restrictionHost.contains("127.0.0.1")) // 127.0.0.1/X might be the only one that works on Windows
         {
            LogTools.warn("This feature might not work on Windows! "
                          + "If you are not receiving data, try it using 127.0.0.1/24 or without setting a subnet restriction.");
         }

         LogTools.info("Scanning interfaces for restriction: {}",  restrictionHost);

         SubnetInfo restrictionSubnetInfo = new SubnetUtils(restrictionHost).getInfo();
         LogTools.debug("Restriction subnet info:\n{}", restrictionSubnetInfo);
         LogTools.info("Restriction address: {}", restrictionSubnetInfo.getAddress());
         LogTools.info("Restriction netmask: {}", restrictionSubnetInfo.getNetmask());

         for (InterfaceAddress interfaceAddress : MACHINE_INTERFACE_ADDRESSES)
         {
            InetAddress address = interfaceAddress.getAddress();

            if (address instanceof Inet4Address)
            {
               short netmaskAsShort = interfaceAddress.getNetworkPrefixLength();

               String interfaceHost = address.getHostAddress();
               SubnetInfo interfaceSubnetInfo = new SubnetUtils(interfaceHost + "/" + netmaskAsShort).getInfo();

               LogTools.debug("Interface Subnet Info: " + interfaceSubnetInfo);
               LogTools.debug("Interface address: " + interfaceSubnetInfo.getAddress());
               LogTools.debug("Interface netmask: " + interfaceSubnetInfo.getNetmask());

               boolean inRange;
               if (SystemUtils.IS_OS_WINDOWS)
               {
                  inRange = interfaceSubnetInfo.isInRange(restrictionSubnetInfo.getAddress()); // This worked on Windows, but not Linux: Doug
               }
               else // Linux and others
               {
                  // This works on Linux. Does not work on Windows. Not tested on Mac.
                  inRange = restrictionSubnetInfo.isInRange(interfaceSubnetInfo.getAddress());
               }

               if (inRange)
               {
                  LogTools.info("Found address in range: {}", address);
                  foundAddressRestriction = address;
                  break;
               }
            }
         }
      }

      defaultDomainID = rtpsDomainID;
      defaultAddressRestriction = foundAddressRestriction;
      if (defaultAddressRestriction != null)
         LogTools.info("Setting IP restriction: {}", defaultAddressRestriction.getHostAddress());
      createParticipant(rtpsDomainID);
   }

   private static List<InterfaceAddress> findInterfaceAddresses()
   {
      try
      {
         return Collections.list(NetworkInterface.getNetworkInterfaces()).stream()
                           .flatMap(networkInterface -> networkInterface.getInterfaceAddresses().stream()).collect(Collectors.toList());
      }
      catch (SocketException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   /**
    * Returns a handle on the singleton instance of the Domain
    * 
    * @return
    */
   public Domain getDomain()
   {
      return domain;
   }

   /**
    * Gets the address to restrict network traffic to when using RTPS.
    * 
    * @return
    */
   public InetAddress getAddressRestriction()
   {
      return defaultAddressRestriction;
   }

   /**
    * Returns a participant attached to the domain.
    */
   public Participant getOrCreateParticipant(int domainID)
   {
      if (!participants.containsKey(domainID))
      {
         createParticipant(domainID);
      }
      return participants.get(domainID);
   }

   /**
    * creates a participant using the default domain ID. The Default domain ID is either loaded from
    * file or randomly generated in the 800-900 range
    * 
    * @return the default participant
    */
   public Participant getDefaultParticipant()
   {
      return getOrCreateParticipant(defaultDomainID);
   }

   /**
    * Creates a participant attached to the domain using the specified domain ID
    * 
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

   int getDomainId()
   {
      return defaultDomainID;
   }

   /**
    * Quick test method to ensure the factory correctly loads the rtps domain ID.
    * 
    * @param args none needed
    */
   public static void main(String[] args)
   {
      new RTPSCommunicationFactory().getDefaultParticipant();
   }
}
