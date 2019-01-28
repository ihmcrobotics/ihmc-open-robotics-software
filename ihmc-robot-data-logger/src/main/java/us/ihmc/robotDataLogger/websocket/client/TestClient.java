package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.net.URI;

import io.netty.bootstrap.Bootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioSocketChannel;
import io.netty.handler.codec.http.DefaultHttpHeaders;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.websocketx.CloseWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshakerFactory;
import io.netty.handler.codec.http.websocketx.WebSocketVersion;
import io.netty.handler.codec.http.websocketx.extensions.compression.WebSocketClientCompressionHandler;

public class TestClient
{
   private static final URI uri = URI.create("ws://127.0.0.1:8080/websocket");
   private final EventLoopGroup group = new NioEventLoopGroup();
   private final Channel ch;

   public TestClient() throws IOException, InterruptedException
   {
      final WebSocketLogClientHandler handler = new WebSocketLogClientHandler(WebSocketClientHandshakerFactory.newHandshaker(uri, WebSocketVersion.V13, null,
                                                                                                                             true, new DefaultHttpHeaders()));

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

      ch = b.connect(uri.getHost(), uri.getPort()).sync().channel();
      handler.handshakeFuture().sync();

   }

   public boolean isOpen()
   {
      return ch.isActive();
   }

   public void close() throws InterruptedException
   {
      ch.writeAndFlush(new CloseWebSocketFrame());
      ch.closeFuture().sync();
   }

   public void cleanup()
   {
      try
      {
         if (ch.isOpen())
         {
            close();
         }
      }
      catch (InterruptedException e)
      {
      } finally
      {
         group.shutdownGracefully();
      }
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {

      final TestClient eveJoystickController = new TestClient();

      Runtime.getRuntime().addShutdownHook(new Thread()
      {
         public void run()
         {
            eveJoystickController.cleanup();
         }
      });
   }

}
