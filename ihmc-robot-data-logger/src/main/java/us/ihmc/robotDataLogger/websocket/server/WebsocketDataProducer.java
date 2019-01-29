package us.ihmc.robotDataLogger.websocket.server;


import java.io.IOException;

import io.netty.bootstrap.ServerBootstrap;
import io.netty.channel.Channel;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.nio.NioServerSocketChannel;
import io.netty.handler.logging.LogLevel;
import io.netty.handler.logging.LoggingHandler;
import io.netty.util.ResourceLeakDetector;
import io.netty.util.ResourceLeakDetector.Level;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.interfaces.DataProducer;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.rtps.CustomLogDataPublisherType;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class WebsocketDataProducer implements DataProducer
{
   public static final int PORT = 8080;
   private final WebsocketDataBroadcaster broadcaster = new WebsocketDataBroadcaster();

   
   private final Object lock = new Object();
   private Channel ch = null;
   private EventLoopGroup bossGroup;
   private EventLoopGroup workerGroup;
   
   private int maximumBufferSize = 0;

   public WebsocketDataProducer(String mainClazz, LogModelProvider logModelProvider, YoVariableServer yoVariableServer, boolean publicBroadcast)
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void remove()
   {
      synchronized(lock)
      {
         try
         {
            ch.close().sync();
            
            bossGroup.shutdownGracefully();
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
   }

   @Override
   public void addCamera(CameraType type, String name, String cameraId)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void announce() throws IOException
   {
      
      synchronized(lock)
      {
         ResourceLeakDetector.setLevel(Level.DISABLED);
         bossGroup = new NioEventLoopGroup(1);
         workerGroup = new NioEventLoopGroup();
         try
         {
            ServerBootstrap b = new ServerBootstrap();
            b.group(bossGroup, workerGroup).channel(NioServerSocketChannel.class).handler(new LoggingHandler(LogLevel.INFO))
             .childHandler(new WebsocketLogServerInitializer(broadcaster, maximumBufferSize));
   
            ch = b.bind(PORT).sync().channel();
   
            System.out.println("Open your web browser and navigate to http://127.0.0.1:" + PORT + '/');
   
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         } 
      }
   }

   @Override
   public void setLog(boolean log)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void publishTimestamp(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public RegistryPublisher createRegistryPublisher(CustomLogDataPublisherType type, PeriodicThreadSchedulerFactory schedulerFactory,
                                                    RegistrySendBufferBuilder builder)
         throws IOException
   {
      WebsocketRegistryPublisher websocketRegistryPublisher = new WebsocketRegistryPublisher(schedulerFactory, builder, broadcaster);
      if(websocketRegistryPublisher.getMaximumBufferSize() > maximumBufferSize)
      {
         maximumBufferSize = websocketRegistryPublisher.getMaximumBufferSize();
      }
      return websocketRegistryPublisher;
   }

   @Override
   public void sendKeepAlive(PeriodicThreadSchedulerFactory schedulerFactory) throws IOException
   {
      // TODO Auto-generated method stub
      
   }

}
