package us.ihmc.robotDataLogger.websocket.server;


import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;

import io.netty.bootstrap.ServerBootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.nio.NioServerSocketChannel;
import io.netty.handler.logging.LogLevel;
import io.netty.handler.logging.LoggingHandler;
import io.netty.util.ResourceLeakDetector;
import io.netty.util.ResourceLeakDetector.Level;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.CameraAnnouncement;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.dataBuffers.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.interfaces.DataProducer;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.util.HandshakeHashCalculator;
import us.ihmc.robotDataLogger.websocket.server.discovery.DataServerLocationBroadcastSender;

/**
 * Implementation of the DataProducer using Websockets
 * 
 * An HTTP server runs on port 8008, and the announcement, handshake, model and resources are 
 * downloadable from that server.
 * 
 * Registry data is send as binary websocket frames, variable changes are received as binary websocket frames. 
 * The underlying encoding format uses DDS IDL/CDR format.
 * 
 * A simple command and echo server using text websocket frames is implemented to send control messages to the server and logger.
 * 
 * Timestamps are send as raw UDP packets after requested over the command server. See {@link us.ihmc.robotDataLogger.websocket.command.DataServerCommand}
 * 
 * @author Jesper Smith
 *
 */
public class WebsocketDataProducer implements DataProducer
{
   private final WebsocketDataBroadcaster broadcaster = new WebsocketDataBroadcaster();
   private final String name;
   private final LogModelProvider logModelProvider;
   private final VariableChangedListener variableChangedListener;
   private final LogAliveListener logAliveListener;

   private final int port;
   
   private final Object lock = new Object();
   private Channel channel = null;
   
   private final EventLoopGroup bossGroup = new NioEventLoopGroup(1);
   
   /**
    * Create a single worker. 
    * 
    * If "writeAndFlush" is called in the eventloop of the outbound channel, no extra objects will be created. 
    * The registryPublisher is scheduled on the main eventloop to avoid having extra threads and delay.
    */
   private final EventLoopGroup workerGroup = new NioEventLoopGroup(1);
   
   private DataServerLocationBroadcastSender broadcastSender;
   
   private Handshake handshake = null;
   
   private int maximumBufferSize = 0;
   
   private final ArrayList<CameraAnnouncement> cameras = new ArrayList<>();
   private final boolean log;
   private final boolean autoDiscoverable;

   private int nextBufferID = 0;

   public WebsocketDataProducer(String name, LogModelProvider logModelProvider, VariableChangedListener variableChangedListener,
                                LogAliveListener logAliveListener, DataServerSettings dataServerSettings)
   {
      this.name = name;
      this.logModelProvider = logModelProvider;
      this.variableChangedListener = variableChangedListener;
      this.logAliveListener = logAliveListener;
      this.port = dataServerSettings.getPort();
      this.log = dataServerSettings.isLogSession();
      this.autoDiscoverable = dataServerSettings.isAutoDiscoverable();
   }

   @Override
   public void remove()
   {
      synchronized(lock)
      {
         try
         {
            if (broadcastSender != null)
               broadcastSender.stop();
            if (broadcaster != null)
               broadcaster.stop();
            
            if (channel != null)
            {
               ChannelFuture closeFuture = channel.close();
               closeFuture.sync();
            }

            if (bossGroup != null)
               bossGroup.shutdownGracefully();

            if (workerGroup != null)
               workerGroup.shutdownGracefully();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }

      }
   }

   @Override
   public void setHandshake(Handshake handshake)
   {
      this.handshake = handshake;
   }

   @Override
   public void addCamera(CameraType type, String name, String cameraId)
   {
      CameraAnnouncement cameraAnnouncement = new CameraAnnouncement();
      cameraAnnouncement.setType(type);
      cameraAnnouncement.setName(name);
      cameraAnnouncement.setIdentifier(cameraId);
      cameras.add(cameraAnnouncement);
   }
   
   private Announcement createAnnouncement() throws UnknownHostException
   {
      Announcement announcement = new Announcement();
      announcement.setName(name);
      announcement.setHostName(InetAddress.getLocalHost().getHostName());
      announcement.setIdentifier("");

      announcement.setLog(log);

      for (CameraAnnouncement camera : cameras)
      {
         announcement.getCameras().add().set(camera);
      }

      String handshakeHash = HandshakeHashCalculator.calculateHash(handshake);
      announcement.setReconnectKey(handshakeHash);

      return announcement;
   }

   @Override
   public void announce() throws IOException
   {
      if(handshake == null)
      {
         throw new RuntimeException("No handshake provided");
      }
      
      Announcement announcement = createAnnouncement();
      DataServerServerContent logServerContent = new DataServerServerContent(announcement, handshake, logModelProvider);
      
      synchronized(lock)
      {
         ResourceLeakDetector.setLevel(Level.DISABLED);
         try
         {
            int numberOfRegistryBuffers = nextBufferID;  // Next buffer ID is incremented the last time a registry was added
            ServerBootstrap serverBootstrap = new ServerBootstrap();
            serverBootstrap.group(bossGroup, workerGroup).channel(NioServerSocketChannel.class).handler(new LoggingHandler(LogLevel.INFO))
                           .childHandler(new WebsocketDataServerInitializer(logServerContent, broadcaster, variableChangedListener, logAliveListener,
                                                                            maximumBufferSize, numberOfRegistryBuffers));

            channel = serverBootstrap.bind(port).sync().channel();
   
            if(autoDiscoverable)
            {
               broadcastSender = new DataServerLocationBroadcastSender(port);
               broadcastSender.start();
            }
            else
            {
               broadcastSender = null;
            }
   
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         } 
      }
   }

   @Override
   public void publishTimestamp(long timestamp)
   {
      broadcaster.publishTimestamp(timestamp);
   }

   @Override
   public RegistryPublisher createRegistryPublisher(CustomLogDataPublisherType type, RegistrySendBufferBuilder builder)
         throws IOException
   {
      WebsocketRegistryPublisher websocketRegistryPublisher = new WebsocketRegistryPublisher(workerGroup, builder, broadcaster, nextBufferID);
      if(websocketRegistryPublisher.getMaximumBufferSize() > maximumBufferSize)
      {
         maximumBufferSize = websocketRegistryPublisher.getMaximumBufferSize();
      }
      nextBufferID++;
      return websocketRegistryPublisher;
   }
}
