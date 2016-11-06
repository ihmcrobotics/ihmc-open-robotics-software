package us.ihmc.robotDataLogger.gui;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.tools.thread.ThreadTools;

public class GUICaptureBroadcast implements Runnable
{
   public static final byte HEADER = 0x42;

   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("GUICaptureBroadcast"));
   private final InetSocketAddress sendAddress;
   private final ByteBuffer ipBuffer = ByteBuffer.allocateDirect(5);
   private final DatagramChannel channel;
   
   private ScheduledFuture<?> future = null;

   public GUICaptureBroadcast(InetAddress myIP, byte[] group) throws IOException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(myIP);
      System.out.println("Announcing UI stream on " + iface);

      this.sendAddress = new InetSocketAddress(InetAddress.getByAddress(group), LogDataProtocolSettings.UI_ANNOUNCE_PORT);

      channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
            .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface);

      ipBuffer.put(HEADER);
      ipBuffer.put(myIP.getAddress());

   }

   public synchronized void start()
   {
      if(future != null)
      {
         future.cancel(false);
      }
      future = scheduler.scheduleAtFixedRate(this, 0, 1, TimeUnit.SECONDS);
   }

   public synchronized void stop()
   {
      if(future != null)
      {
         future.cancel(false);
      }
   }

   @Override
   public void run()
   {
      try
      {
         ipBuffer.clear();
         channel.send(ipBuffer, sendAddress);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException(e);
      }
   }

   public static InetAddress getIP(NetworkInterface iface, InetAddress inetAddress) throws IOException
   {
      InetSocketAddress receiveAddress = new InetSocketAddress(inetAddress, LogDataProtocolSettings.UI_ANNOUNCE_PORT);

      DatagramChannel channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(receiveAddress);
      MembershipKey receiveKey = channel.join(inetAddress, iface);
      byte header = 0; 
      byte[] ip = new byte[4];
      while(header != HEADER)
      {
         ByteBuffer announceBuffer = ByteBuffer.allocateDirect(5); 
         channel.receive(announceBuffer);
         announceBuffer.flip();
         header = announceBuffer.get();
         announceBuffer.get(ip);
      }
      
      receiveKey.drop();
      channel.close();
      return InetAddress.getByAddress(ip);
      

   }
}
