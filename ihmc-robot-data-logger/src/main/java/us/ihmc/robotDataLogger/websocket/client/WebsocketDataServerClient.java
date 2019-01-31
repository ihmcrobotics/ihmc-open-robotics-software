package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import io.netty.bootstrap.Bootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelOption;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioSocketChannel;
import io.netty.handler.codec.http.DefaultHttpHeaders;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshakerFactory;
import io.netty.handler.codec.http.websocketx.WebSocketVersion;
import io.netty.handler.codec.http.websocketx.extensions.compression.WebSocketClientCompressionHandler;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.dataBuffers.RegistryConsumer;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

public class WebsocketDataServerClient
{
   private final EventLoopGroup group = new NioEventLoopGroup();
   private final RegistryConsumer consumer;

   private final Channel ch;

   public WebsocketDataServerClient(HTTPDataServerDescription target, IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient, RTPSDebugRegistry rtpsDebugRegistry) throws IOException
   {
      URI uri;
      try
      {
         uri = new URI("ws://" + target.getHost() + ":" + target.getPort() + "/websocket");
      }
      catch (URISyntaxException e)
      {
         throw new IOException(e);
      }
      
      this.consumer = new RegistryConsumer(parser, yoVariableClient, rtpsDebugRegistry);
      

      final WebSocketDataServerClientHandler handler = new WebSocketDataServerClientHandler(WebSocketClientHandshakerFactory.newHandshaker(uri,
                                                                                                                                           WebSocketVersion.V13,
                                                                                                                                           null, true,
                                                                                                                                           new DefaultHttpHeaders()), yoVariableClient, consumer);

      Bootstrap b = new Bootstrap();
      b.group(group).channel(NioSocketChannel.class).handler(new ChannelInitializer<SocketChannel>()
      {
         @Override
         protected void initChannel(SocketChannel ch)
         {
            ChannelPipeline p = ch.pipeline();
            p.addLast(new HttpClientCodec(), new HttpObjectAggregator(65536), WebSocketClientCompressionHandler.INSTANCE, handler);
         }
      });

      b.option(ChannelOption.CONNECT_TIMEOUT_MILLIS, 1000);

      try
      {
         ch = b.connect(uri.getHost(), uri.getPort()).syncUninterruptibly().channel();
      }
      catch (Exception e)
      {
         throw new IOException(e);
      }

   }

   
   public boolean isActive()
   {
      return ch.isActive();
   }


   public void close()
   {
      ch.close();
   }
}
