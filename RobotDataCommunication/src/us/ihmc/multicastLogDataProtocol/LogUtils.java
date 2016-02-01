package us.ihmc.multicastLogDataProtocol;

import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Enumeration;

public class LogUtils
{
   
   public static InetAddress getByName(String address)
   {
      try
      {
         return InetAddress.getByName(address);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }
   public static InetAddress getByAddress(byte[] address)
   {
      try
      {
         return InetAddress.getByAddress(address);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public static NetworkInterface getMyInterface(String hostOnNetwork)
   {
      try
      {
         return NetworkInterface.getByInetAddress(LogUtils.getMyIP(hostOnNetwork));
      }
      catch (SocketException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static InetAddress getMyIP(String host)
   {
      try
      {
         return getMyIP(InetAddress.getByName(host));
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static InetAddress getMyIP(byte[] address)
   {
      try
      {
         return getMyIP(InetAddress.getByAddress(address));
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }

   }

   /**
    * Finds the IP address of this computer that can resolve ipOnNetwork within the current subnet. Returns the network with the most specific subnet.
    * 
    * @param ipOnNetwork
    * @return my IP address
    */
   public static InetAddress getMyIP(InetAddress ipOnNetwork)
   {
      try
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
      catch (SocketException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static int addressToInt(InetAddress address)
   {
      return ByteBuffer.wrap(address.getAddress()).getInt();
   }

   public static void main(String[] args) throws UnknownHostException
   {
   }
}
