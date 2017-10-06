package us.ihmc.robotDataLogger.util;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Enumeration;

public class LogUtils
{

   public static InetAddress getByName(String address) throws UnknownHostException
   {
      return InetAddress.getByName(address);
   }

   public static InetAddress getByAddress(byte[] address) throws UnknownHostException
   {
      return InetAddress.getByAddress(address);
   }

   public static NetworkInterface getMyInterface(String hostOnNetwork) throws IOException
   {
      return NetworkInterface.getByInetAddress(LogUtils.getMyIP(hostOnNetwork));
   }

   public static InetAddress getMyIP(String host) throws IOException
   {
      return getMyIP(InetAddress.getByName(host));
   }

   public static InetAddress getMyIP(byte[] address) throws IOException
   {
      return getMyIP(InetAddress.getByAddress(address));

   }

   /**
    * Finds the IP address of this computer that can resolve ipOnNetwork within the current subnet. Returns the network with the most specific subnet.
    * 
    * @param ipOnNetwork
    * @return my IP address
    */
   public static InetAddress getMyIP(InetAddress ipOnNetwork) throws IOException
   {
      Enumeration<NetworkInterface> ifaces = NetworkInterface.getNetworkInterfaces();

      InetAddress myIp = null;
      int networkPrefixLength = 0;

      while (ifaces.hasMoreElements())
      {
         for (InterfaceAddress address : ifaces.nextElement().getInterfaceAddresses())
         {
            if (address.getAddress().getAddress().length == 4)
            {
               if (addressToInt(address.getAddress()) == addressToInt(ipOnNetwork))
               {
                  networkPrefixLength = 32;
                  myIp = address.getAddress();
               }

               int netmask = ~(address.getNetworkPrefixLength() != 0 ? (0xFFFFFFFF >>> address.getNetworkPrefixLength()) : 0);
               if ((addressToInt(address.getAddress()) & netmask) == (addressToInt(ipOnNetwork) & netmask))
               {
                  if (address.getNetworkPrefixLength() > networkPrefixLength)
                  {
                     networkPrefixLength = address.getNetworkPrefixLength();
                     myIp = address.getAddress();
                  }
               }
            }
         }
      }

      if (myIp != null)
      {
         return myIp;
      }
      else
      {
         throw new RuntimeException("Cannot reach IP " + ipOnNetwork);
      }
   }

   public static int addressToInt(InetAddress address)
   {
      return ByteBuffer.wrap(address.getAddress()).getInt();
   }

}
