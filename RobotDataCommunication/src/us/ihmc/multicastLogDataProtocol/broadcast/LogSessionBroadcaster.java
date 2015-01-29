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
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.security.SecureRandom;
import java.util.Arrays;
import java.util.Random;

import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.robotDataCommunication.gui.GUICaptureStreamer;
import us.ihmc.robotDataCommunication.logger.LogSettings;

public class LogSessionBroadcaster extends Thread
{
   public static final byte[] announceGroupAddress = { (byte) 239, (byte) 255, (byte) 24, (byte) 1 };
   private final int GROUP_REQUEST_TIMEOUT = 100;
   private final int GROUP_REQUEST_ATTEMPTS = 10;
   private final int GROUP_ANNOUNCE_RATE = 500;

   private final NetworkInterface iface;
   private final InetSocketAddress controlAddress;
   private final LogSettings logSettings;
   
   private final long sessionID;
   private final String className;

   private final InetSocketAddress address = new InetSocketAddress(LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);
   private final InetAddress announceGroup;
   private final InetSocketAddress sendAddress;

   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);
   private final ByteBuffer sendBuffer = ByteBuffer.allocate(65535);

   private final byte[] multicastGroupAddress = new byte[4];
   private int dataPort;

   private DatagramChannel channel;
   private MembershipKey receiveKey;
   private final Selector selector;
   private SelectionKey key;

   public LogSessionBroadcaster(InetSocketAddress controlAddress, Class<?> clazz, LogSettings logSettings)
   {
      try
      {
         this.iface = NetworkInterface.getByInetAddress(controlAddress.getAddress());
         System.out.println(iface);
         this.controlAddress = controlAddress;
         this.logSettings = logSettings;
         this.className = clazz.getSimpleName();
         long mac;
         if (iface.isLoopback())
         {
            mac = 0xFFFFFFFFFFFFl;
         }
         else
         {
            mac = new BigInteger(1, iface.getHardwareAddress()).longValue();
         }
         long uid = createTempSessionID() << 48l;
         sessionID = mac | uid;

         this.announceGroup = InetAddress.getByAddress(announceGroupAddress);
         this.sendAddress = new InetSocketAddress(announceGroup, LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);
         this.selector = Selector.open();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
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
      canIHazRequest.setVideoStream(logSettings.getVideoStream().getAddress());
      canIHazRequest.setVideoPort(GUICaptureStreamer.PORT);
      canIHazRequest.setCameras(logSettings.getCameras());
      canIHazRequest.setLog(logSettings.isLog());
      canIHazRequest.setName(className);

      setRandomGroupId();
      try
      {
         channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
               .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface).bind(address);
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
         canIHazRequest.setDataPort(dataPort);
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
                     if (reply.getSessionID() != sessionID && (Arrays.equals(reply.getGroup(), multicastGroupAddress) || reply.getDataPort() == dataPort))
                     {
                        System.out.println(Arrays.toString(multicastGroupAddress) + ":" + dataPort + " is already taken.");
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
      announcement.setDataPort(dataPort);
      announcement.setControlIP(controlAddress.getAddress().getAddress());
      announcement.setControlPort((short) controlAddress.getPort());
      announcement.setVideoStream(logSettings.getVideoStream().getAddress());
      announcement.setVideoPort(GUICaptureStreamer.PORT);
      announcement.setCameras(logSettings.getCameras());
      announcement.setLog(logSettings.isLog());
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
                           && (Arrays.equals(reply.getGroup(), multicastGroupAddress) || reply.getDataPort() == dataPort))
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

   public InetAddress getGroup()
   {
      try
      {
         return InetAddress.getByAddress(multicastGroupAddress);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public int getPort()
   {
      return dataPort;
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
      dataPort = LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START + (multicastGroupAddress[3] & 0xFF);

      System.out.println("Trying " + (multicastGroupAddress[0] & 0xFF) + "." + (multicastGroupAddress[1] & 0xFF) + "." + (multicastGroupAddress[2] & 0xFF)
            + "." + (multicastGroupAddress[3] & 0xFF));
   }

   public static void main(String[] args) throws SocketException, IOException, InterruptedException
   {
      InetSocketAddress controlAddress = new InetSocketAddress("127.0.0.1", LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);
      LogSessionBroadcaster logSessionAnnounce = new LogSessionBroadcaster(controlAddress, LogSessionBroadcaster.class, LogSettings.SIMULATION);
      logSessionAnnounce.start();
      logSessionAnnounce.join();
   }

   public NetworkInterface getInterface()
   {
      return iface;
   }

   public void close()
   {
      interrupt();
      try
      {
         join();
      }
      catch (InterruptedException e)
      {
      }
   }
}
