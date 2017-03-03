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
import java.nio.file.Path;
import java.security.SecureRandom;
import java.util.Arrays;
import java.util.Random;

import us.ihmc.commons.nio.PathTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.tools.io.printing.PrintTools;

public class LogSessionBroadcaster extends Thread
{
   public static final byte[] announceGroupAddress = { (byte) 239, (byte) 255, (byte) 24, (byte) 1 };
   private final int GROUP_REQUEST_TIMEOUT = 50;
   private final int GROUP_REQUEST_ATTEMPTS = 5;
   private final int GROUP_ANNOUNCE_RATE = 500;

   private final NetworkInterface iface;
   private final InetSocketAddress controlAddress;
   private final InetAddress dataAddress;
   private int dataPort;
   
   private final LogSettings logSettings;
   
   private final long sessionID;
   private final String className;

   private final InetSocketAddress address = new InetSocketAddress(LogDataProtocolSettings.UI_ANNOUNCE_PORT);
   private final InetAddress announceGroup;
   private final InetSocketAddress sendAddress;

   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);
   private final ByteBuffer sendBuffer = ByteBuffer.allocate(65535);


   private DatagramChannel channel;
   private MembershipKey receiveKey;
   private final Selector selector;
   private SelectionKey key;
   
   private final byte[] cameras;

   public LogSessionBroadcaster(InetSocketAddress controlAddress, InetAddress dataAddress, String className, LogSettings logSettings)
   {
      try
      {
         this.iface = NetworkInterface.getByInetAddress(controlAddress.getAddress());
         PrintTools.info(this, "Announcing logging session on: " + iface);
         this.controlAddress = controlAddress;
         this.dataAddress = dataAddress;
         this.logSettings = logSettings;
         this.className = className;
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
         
         String cameraString;
         
         if(NetworkParameters.hasKey(NetworkParameterKeys.loggedCameras))
         {
            cameraString = NetworkParameters.getHost(NetworkParameterKeys.loggedCameras);
         }
         else
         {
            cameraString = null;
         }
            
         if(cameraString != null && !cameraString.trim().isEmpty())
         {
            String[] split = cameraString.split(",");
            cameras = new byte[split.length];
            for(int i = 0; i < split.length; i++)
            {
               try
               {
                  byte camera = Byte.parseByte(split[i].trim());
                  if(camera >= 0 && camera <= 127)
                  {
                     cameras[i] = camera;
                  }
                  else
                  {
                     throw new NumberFormatException();
                  }
               }
               catch(NumberFormatException e)
               {
                  throw new RuntimeException("Invalid camera id: " + split[i] +". Valid camera ids are 0-127");
               }
            }
         }
         else
         {
            cameras = new byte[0];
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void requestPort()
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
      canIHazRequest.setDataIP(dataAddress.getAddress());
      if(logSettings.getVideoStream() != null)
      {
         canIHazRequest.setVideoStream(logSettings.getVideoStream().getAddress());
         canIHazRequest.setVideoPort(LogDataProtocolSettings.UI_DATA_PORT);
      }
      canIHazRequest.setCameras(cameras);
      canIHazRequest.setLog(logSettings.isLog());
      canIHazRequest.setName(className);

		setRandomPort();
      try
      {
         canIHazRequest.setDataPort(dataPort);
      
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
                     if (reply.getSessionID() != sessionID && (Arrays.equals(reply.getDataIP(), dataAddress.getAddress()) && reply.getDataPort() == dataPort))
                     {
                        System.out.println(Arrays.toString(dataAddress.getAddress()) + ":" + dataPort + " is already taken.");
                        setRandomPort();
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

   }

   @Override
   public void run()
   {
      AnnounceRequest reply = new AnnounceRequest();
      AnnounceRequest announcement = new AnnounceRequest();
      announcement.setType(AnnounceRequest.AnnounceType.ANNOUNCE);
      announcement.setSessionID(sessionID);
      announcement.setDataIP(dataAddress.getAddress());
      announcement.setDataPort(dataPort);
      announcement.setControlIP(controlAddress.getAddress().getAddress());
      announcement.setControlPort((short) controlAddress.getPort());
      if(logSettings.getVideoStream() != null)
      {
         announcement.setVideoStream(logSettings.getVideoStream().getAddress());
         announcement.setVideoPort(LogDataProtocolSettings.UI_ANNOUNCE_PORT);
      }
      announcement.setCameras(cameras);
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
                           && (Arrays.equals(reply.getDataIP(), dataAddress.getAddress()) && reply.getDataPort() == dataPort))
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
      Path temporaryDirectoryPath = PathTools.systemTemporaryDirectory();

      SecureRandom random = new SecureRandom();

      int sessionID;
      File sessionFile;
      do
      {
         sessionID = random.nextInt(65535);
         sessionFile = temporaryDirectoryPath.resolve(prefix + sessionID).toFile();
      }
      while (!sessionFile.createNewFile());
      sessionFile.deleteOnExit();

      return sessionID;
   }

   public void setRandomPort()
   {
      dataPort = LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START + (new Random().nextInt(253) + 1);

      PrintTools.info("Trying port " + dataPort);
   }

   public static void main(String[] args) throws SocketException, IOException, InterruptedException
   {
      InetSocketAddress controlAddress = new InetSocketAddress("127.0.0.1", LogDataProtocolSettings.LOG_DATA_ANNOUNCE_PORT);
      InetAddress dataAddress = InetAddress.getByName("127.0.0.1");
      LogSessionBroadcaster logSessionAnnounce = new LogSessionBroadcaster(controlAddress, dataAddress, LogSessionBroadcaster.class.getSimpleName(), LogSettings.SIMULATION);
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

   public int getPort()
   {
      return dataPort;
   }
}
