package us.ihmc.multicastLogDataProtocol.broadcast;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketAddress;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;

import org.apache.commons.lang3.SystemUtils;

import gnu.trove.impl.Constants;
import gnu.trove.iterator.TLongObjectIterator;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest.AnnounceType;

public class LogSessionBroadcastClient extends Thread
{
   private static final long TIMEOUT = 20000;

   private final InetAddress announceGroup = InetAddress.getByAddress(LogSessionBroadcaster.announceGroupAddress);
   private final InetSocketAddress address;

   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

   private final TLongObjectHashMap<TimestampedEntryHolder> logSessions = new TLongObjectHashMap<>(Constants.DEFAULT_CAPACITY, Constants.DEFAULT_LOAD_FACTOR, Long.MIN_VALUE);
   private final LogBroadcastListener listener;
   private final NetworkInterface iface;

   public LogSessionBroadcastClient(NetworkInterface iface, LogBroadcastListener listener) throws IOException
   {
      super("LogSessionBroadcastClient");
      this.iface = iface;
      this.listener = listener;
      
      if(SystemUtils.IS_OS_WINDOWS)
      {
         // Windows cannot bind to multicast group IPs
         address = new InetSocketAddress(LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);
      }
      else
      {
         address = new InetSocketAddress(announceGroup, LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);         
      }
   }

   @Override
   public void run()
   {
      try
      {
         DatagramChannel channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(address);
         MembershipKey receiveKey = channel.join(announceGroup, iface);
         channel.configureBlocking(false);
         Selector selector = Selector.open();
         SelectionKey key = channel.register(selector, SelectionKey.OP_READ);

         AnnounceRequest reply = new AnnounceRequest();

         while (!isInterrupted())
         {
            if (selector.select(TIMEOUT / 10) > 0)
            {
               selector.selectedKeys().remove(key);
               if (key.isReadable())
               {
                  receiveBuffer.clear();
                  SocketAddress source = channel.receive(receiveBuffer);
                  receiveBuffer.flip();
                  if(reply.readHeader(receiveBuffer) && reply.readVariableLengthData(receiveBuffer))
                  {
                     if (reply.getType() == AnnounceType.ANNOUNCE)
                     {
                        handleRequest(reply);
                     }
                  }
                  else
                  {
                     System.err.println("Received invalid packet from " + source);
                  }
               }
            }
            removeDisconnected();
         }

         receiveKey.drop();
         channel.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void removeDisconnected()
   {
      long time = System.nanoTime();

      TLongObjectIterator<TimestampedEntryHolder> it = logSessions.iterator();
      
      while (it.hasNext())
      {
         it.advance();
         if ((time - it.value().getLastSeen()) > (TIMEOUT * 1000000))
         {
            listener.logSessionWentOffline(it.value().getAnnounceRequest());
            it.remove();
         }

      }
   }

   private void handleRequest(AnnounceRequest reply)
   {
      TimestampedEntryHolder timestamp = logSessions.get(reply.getSessionID());
      if (timestamp != null)
      {
         timestamp.setLastSeen();
      }
      else
      {
         TimestampedEntryHolder value = new TimestampedEntryHolder(reply);
         if(logSessions.put(reply.getSessionID(), value) == null)
         {
            listener.logSessionCameOnline(value.getAnnounceRequest());
         }
      }
   }

   private static class TimestampedEntryHolder
   {
      private final AnnounceRequest announce;
      private long lastSeen;

      public TimestampedEntryHolder(AnnounceRequest announce)
      {
         this.announce = new AnnounceRequest(announce);
         setLastSeen();
      }

      public void setLastSeen()
      {
         this.lastSeen = System.nanoTime();
      }

      public long getLastSeen()
      {
         return this.lastSeen;
      }
      
      public AnnounceRequest getAnnounceRequest()
      {
         return announce;
      }
   }
}
