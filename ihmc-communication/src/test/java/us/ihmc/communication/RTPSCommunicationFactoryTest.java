package us.ihmc.communication;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.jupiter.api.Test;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

public class RTPSCommunicationFactoryTest extends NetworkParametersTest
{
   @Test
   @Override
   public void testNoFile()
   {
      super.testNoFile();

      for (int i = 0; i < 90; ++i)
      {
         RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
         assertTrue(factory.getDomainId() > 199 && factory.getDomainId() < 230);
         assertNotNull(factory.getAddressRestriction());
         assertEquals(0, factory.getAddressRestriction().length);
      }
   }

   @Test
   @Override
   public void testEmptyFile()
   {
      super.testEmptyFile();

      for (int i = 0; i < 90; ++i)
      {
         RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
         assertTrue(factory.getDomainId() > 199 && factory.getDomainId() < 230);
         assertNotNull(factory.getAddressRestriction());
         assertEquals(0, factory.getAddressRestriction().length);
      }
   }

   @Test
   @Override
   public void testLoadingRTPSDomainID()
   {
      int testDomainID = RANDOM.nextInt(230);
      testLoadingRTPSDomainID(testDomainID);

      RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
      assertEquals(testDomainID, factory.getDomainId());
   }

   @Test
   @Override
   public void testLoadingRTPSDomainRestrictions()
   {
      InterfaceAddress[] availableAddresses = getAvailableAddresses();

      testLoadingRTPSDomainRestrictions(interfaceAddressToString(availableAddresses));

      RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
      assertEquals(availableAddresses.length, factory.getAddressRestriction().length);
   }

   @Test
   public void testLoopbackOnlyRestriction()
   {
      InterfaceAddress[] availableAddresses = getAvailableAddresses();
      InterfaceAddress loopbackAddress = Arrays.stream(availableAddresses).filter(address -> address.getAddress().isLoopbackAddress()).findAny().orElse(null);
      assumeTrue(loopbackAddress != null);

      testLoadingRTPSDomainRestrictions(interfaceAddressToString(loopbackAddress));

      RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
      assertEquals(1, factory.getAddressRestriction().length);
      assertEquals(InetAddress.getLoopbackAddress(), factory.getAddressRestriction()[0]);
   }

   @Test
   public void testAllExclusiveRestriction()
   {
      testLoadingRTPSDomainRestrictions("0.0.0.0/32");

      RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
      assertEquals(0, factory.getAddressRestriction().length);
   }

   @Test
   public void testAllInclusiveRestriction()
   {
      InterfaceAddress[] availableAddresses = getAvailableAddresses();

      testLoadingRTPSDomainRestrictions("0.0.0.0/0");

      RTPSCommunicationFactory factory = new RTPSCommunicationFactory();
      assertEquals(availableAddresses.length, factory.getAddressRestriction().length);
   }

   private String[] interfaceAddressToString(InterfaceAddress... addresses)
   {
      String[] addressesAsStrings = new String[addresses.length];
      for (int i = 0; i < addresses.length; ++i)
         addressesAsStrings[i] = interfaceAddressToString(addresses[i]);
      return addressesAsStrings;
   }

   private String interfaceAddressToString(InterfaceAddress address)
   {
      return address.getAddress().getHostAddress() + "/" + address.getNetworkPrefixLength();
   }

   private InterfaceAddress[] getAvailableAddresses()
   {
      MutableBoolean success = new MutableBoolean(true);
      InterfaceAddress[] addresses;
      try
      {
         addresses = NetworkInterface.networkInterfaces()
                                     .flatMap(networkInterface -> networkInterface.getInterfaceAddresses().stream())
                                     .filter(interfaceAddress -> interfaceAddress.getAddress() instanceof Inet4Address)
                                     .toArray(InterfaceAddress[]::new);
      }
      catch (SocketException e)
      {
        success.setFalse();
        addresses = null;
      }

      assumeTrue(success.getValue());
      return addresses;
   }
}
