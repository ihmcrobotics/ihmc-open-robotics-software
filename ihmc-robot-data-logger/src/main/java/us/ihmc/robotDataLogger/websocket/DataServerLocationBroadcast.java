package us.ihmc.robotDataLogger.websocket;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.Host;
import us.ihmc.robotDataLogger.StaticHostList;

/**
 * Common functions for the DataServerLocationBroadcast client and sender
 * 
 * @author Jesper Smith
 *
 */
public abstract class DataServerLocationBroadcast
{
   private static final String PORT_MESSAGE_HEADER = "DataServerPort";
   
   public static class PortPOJO
   {
      public String header;
      public int port;
      
      public PortPOJO()
      {
         
      }
      
      public PortPOJO(int port)
      {
         this.header = PORT_MESSAGE_HEADER;
         this.port = port;
      }
   }
   
   
   public static final String announceGroupAddress = "239.255.24.1";
   public static final int announcePort = 55241;
   public static final int MAXIMUM_MESSAGE_SIZE = 1472;


   /**
    * Get a list of all external IP addresses.
    * 
    * 
    * @return List of IP addresses
    * @throws IOException 
    */
   protected static StaticHostList getMyNetworkAddresses(int dataServerPort) throws IOException
   {
      StaticHostList addresses = new StaticHostList();
   
      for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces()))
      {
         if (iface.isUp())
         {
            for (InetAddress addr : Collections.list(iface.getInetAddresses()))
            {
               if(addr instanceof Inet4Address)
               {
                  if(!addr.isLoopbackAddress())
                  {
                     Host host = addresses.getHosts().add();
                     host.setHostname(addr.getHostAddress());
                     host.setPort(dataServerPort);
                  }
               }
            }
   
         }
      }
   
      return addresses;
   }

   protected static List<MulticastSocket> getSocketChannelList(int bindPort) throws IOException
   {
      List<MulticastSocket> sockets = new ArrayList<MulticastSocket>();
      for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces()))
      {
         try
         {
            if(iface.isUp() && !iface.isLoopback() && iface.supportsMulticast() && !iface.isVirtual())
            {
               MulticastSocket socket = new MulticastSocket(bindPort);
               socket.setNetworkInterface(iface);
               sockets.add(socket);
            }
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot add " + iface.getDisplayName() + " to list of broadcast sockets. " + e.getMessage());
         }
      }
      
      return sockets;
   }

   protected static String createMessage(int port) throws JsonProcessingException
   {
      ObjectMapper mapper = new ObjectMapper(new JsonFactory());
      PortPOJO portPOJO = new PortPOJO(port);
      return mapper.writeValueAsString(portPOJO);
   }
   
   protected static int parseMessage(String message, ObjectMapper mapper) throws IOException
   {
      PortPOJO portPOJO = mapper.readValue(message, PortPOJO.class);
      if(PORT_MESSAGE_HEADER.equals(portPOJO.header))
      {
         return portPOJO.port;         
      }
      else
      {
         throw new JsonParseException(null, "Invalid header.");
      }
      
   }
}
