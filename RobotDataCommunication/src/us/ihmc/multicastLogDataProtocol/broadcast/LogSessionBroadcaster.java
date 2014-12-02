package us.ihmc.multicastLogDataProtocol.broadcast;

import java.io.File;
import java.io.IOException;
import java.math.BigInteger;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.security.SecureRandom;
import java.util.Arrays;
import java.util.Random;

import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;

public class LogSessionBroadcaster extends Thread
{
   public static final byte[] announceGroupAddress = { (byte) 239, (byte) 255, (byte) 24, (byte) 1 };
   private final int GROUP_REQUEST_TIMEOUT = 100;
   private final int GROUP_REQUEST_ATTEMPTS = 10;
   private final int GROUP_ANNOUNCE_RATE = 500;

   private final NetworkInterface iface;
   private final InetSocketAddress controlAddress;

   
   private final long sessionID;
   private final String className;

   private final InetSocketAddress address = new InetSocketAddress(LogDataProtocolSettings.LOG_DATA_PORT);
   private final InetAddress announceGroup = InetAddress.getByAddress(announceGroupAddress);
   private final InetSocketAddress sendAddress = new InetSocketAddress(announceGroup, LogDataProtocolSettings.LOG_DATA_PORT);

   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);
   private final ByteBuffer sendBuffer = ByteBuffer.allocate(65535);

   private final byte[] multicastGroupAddress = new byte[4];

   
   private DatagramChannel channel;
   private MembershipKey receiveKey;
   private final Selector selector = Selector.open();
   private SelectionKey key;

   public LogSessionBroadcaster(InetSocketAddress controlAddress, Class<?> clazz) throws IOException
   {
      this.iface = NetworkInterface.getByInetAddress(controlAddress.getAddress());
      this.controlAddress = controlAddress;
      this.className = clazz.getSimpleName();
      long mac = new BigInteger(1, iface.getHardwareAddress()).longValue();
      long uid = createTempSessionID() << 48l;
      sessionID = mac | uid;
   }

   public void start()
   {
      if (getState() != Thread.State.NEW)
      {
         throw new IllegalThreadStateException();
      }
      AnnounceRequest reply = new AnnounceRequest();
      AnnounceRequest canIHazRequest = new AnnounceRequest();
      canIHazRequest.setType(AnnounceRequest.AnnounceType.CAN_I_HAZ);
      canIHazRequest.setSessionID(sessionID);
      canIHazRequest.setControlIP(controlAddress.getAddress().getAddress());
      canIHazRequest.setControlPort((short) controlAddress.getPort());
      canIHazRequest.setName(className);

      setRandomGroupId();
      try
      {
         channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(address);
         channel.socket().setReceiveBufferSize(65535);
         receiveKey = channel.join(announceGroup, iface);
         channel.configureBlocking(false);
         key = channel.register(selector, SelectionKey.OP_READ);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      for (int i = 0; i < GROUP_REQUEST_ATTEMPTS; i++)
      {
         canIHazRequest.setGroup(multicastGroupAddress);
         canIHazRequest.createRequest(sendBuffer);
         try
         {
            channel.send(sendBuffer, sendAddress);
            long start = System.currentTimeMillis();
            long elapsed;
            while ((elapsed = System.currentTimeMillis() - start) < GROUP_REQUEST_TIMEOUT)
            {
               if (selector.select(GROUP_REQUEST_TIMEOUT - elapsed) > 0)
               {
                  selector.selectedKeys().remove(key);
                  if (key.isReadable())
                  {
                     receiveBuffer.clear();
                     channel.receive(receiveBuffer);
                     receiveBuffer.flip();
                     reply.readHeader(receiveBuffer);
                     if (reply.getSessionID() != sessionID && Arrays.equals(reply.getGroup(), multicastGroupAddress))
                     {
                        System.out.println(Arrays.toString(multicastGroupAddress) + " is already taken.");
                        setRandomGroupId();
                        i = 0;
                        break;
                     }
                  }
               }
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      super.start();
   }

   @Override
   public void run()
   {
      AnnounceRequest reply = new AnnounceRequest();
      AnnounceRequest announcement = new AnnounceRequest();
      announcement.setType(AnnounceRequest.AnnounceType.ANNOUNCE);
      announcement.setSessionID(sessionID);
      announcement.setGroup(multicastGroupAddress);
      announcement.setControlIP(controlAddress.getAddress().getAddress());
      announcement.setControlPort((short) controlAddress.getPort());
      announcement.setName(className);
      announcement.createRequest(sendBuffer);

      while (!isInterrupted())
      {
         try
         {
            channel.send(sendBuffer, sendAddress);
            sendBuffer.flip();

            long start = System.currentTimeMillis();
            long elapsed;
            while ((elapsed = System.currentTimeMillis() - start) < GROUP_ANNOUNCE_RATE)
            {
               if (selector.select(GROUP_ANNOUNCE_RATE - elapsed) > 0)
               {
                  selector.selectedKeys().remove(key);
                  if (key.isReadable())
                  {
                     receiveBuffer.clear();
                     channel.receive(receiveBuffer);
                     receiveBuffer.flip();
                     reply.readHeader(receiveBuffer);
                     if (reply.getSessionID() != sessionID && reply.getType() == AnnounceRequest.AnnounceType.CAN_I_HAZ
                           && Arrays.equals(reply.getGroup(), multicastGroupAddress))
                     {
                        break;
                     }
                  }
               }

            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
      
      key.cancel();
      receiveKey.drop();
      try
      {
         channel.close();
      }
      catch (IOException e)
      {
      }

   }

   public long getSessionID()
   {
      return sessionID;
   }

   private static long createTempSessionID() throws IOException
   {
      String prefix = "LogSession";
      String tmpDir = System.getProperty("java.io.tmpdir");

      SecureRandom random = new SecureRandom();

      int sessionID;
      File sessionFile;
      do
      {
         sessionID = random.nextInt(65535);
         sessionFile = new File(tmpDir, prefix + sessionID);
      }
      while (!sessionFile.createNewFile());
      sessionFile.deleteOnExit();

      return sessionID;
   }

   public void setRandomGroupId()
   {
      multicastGroupAddress[0] = announceGroupAddress[0];
      multicastGroupAddress[1] = announceGroupAddress[1];
      multicastGroupAddress[2] = announceGroupAddress[2];
      multicastGroupAddress[3] = (byte) (new Random().nextInt(253) + 1);

      System.out.println("Trying " + (multicastGroupAddress[0] & 0xFF) + "." + (multicastGroupAddress[1] & 0xFF) + "." + (multicastGroupAddress[2] & 0xFF)
            + "." + (multicastGroupAddress[3] & 0xFF));
   }

   public static void main(String[] args) throws SocketException, IOException, InterruptedException
   {
      InetSocketAddress controlAddress = new InetSocketAddress("10.6.192.1", LogDataProtocolSettings.LOG_DATA_PORT);
      LogSessionBroadcaster logSessionAnnounce = new LogSessionBroadcaster(controlAddress, LogSessionBroadcaster.class);
      logSessionAnnounce.start();
      logSessionAnnounce.join();
   }
}
