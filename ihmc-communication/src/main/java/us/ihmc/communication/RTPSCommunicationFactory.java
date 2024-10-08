package us.ihmc.communication;

import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.net.util.SubnetUtils;
import org.apache.commons.net.util.SubnetUtils.SubnetInfo;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Loads the Domain ID and Subnet Restriction from ~/.ihmc/IHMCNetworkParameters.ini
 */
public class RTPSCommunicationFactory
{
   private static final int START_OF_RANDOM_DOMAIN_RANGE = 200;
   private static final List<InterfaceAddress> MACHINE_INTERFACE_ADDRESSES = findInterfaceAddresses();

   private final int defaultDomainID;
   private final InetAddress[] defaultAddressRestrictions;

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

      Set<InetAddress> foundAddressRestrictions = new HashSet<>();

      String restrictionHostString;
      if (MACHINE_INTERFACE_ADDRESSES != null
       && NetworkParameters.hasKey(NetworkParameterKeys.RTPSSubnet)
       && !(restrictionHostString = NetworkParameters.getHost(NetworkParameterKeys.RTPSSubnet)).isEmpty())
      {
         if (SystemUtils.IS_OS_WINDOWS && !restrictionHostString.contains("127.0.0.1")) // 127.0.0.1/X might be the only one that works on Windows
         {
            LogTools.warn("This feature might not work on Windows! "
                          + "If you are not receiving data, try it using 127.0.0.1/24 or without setting a subnet restriction.");
         }

         LogTools.info("Scanning interfaces for restriction: {}",  restrictionHostString);

         String[] restrictionHostList = restrictionHostString.split("\\s*,\\s*");
         for (String restrictionHost : restrictionHostList)
         {
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
                     foundAddressRestrictions.add(address);
                  }
               }
            }
         }
      }

      defaultDomainID = rtpsDomainID;
      defaultAddressRestrictions = foundAddressRestrictions.toArray(InetAddress[]::new);
      if (defaultAddressRestrictions.length > 0)
         LogTools.info("Setting IP restriction: {}", Arrays.stream(defaultAddressRestrictions).map(InetAddress::getHostAddress).toList());
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
    * Gets the address to restrict network traffic to when using RTPS.
    * 
    * @return
    */
   public InetAddress[] getAddressRestriction()
   {
      return defaultAddressRestrictions;
   }

   int getDomainId()
   {
      return defaultDomainID;
   }
}
