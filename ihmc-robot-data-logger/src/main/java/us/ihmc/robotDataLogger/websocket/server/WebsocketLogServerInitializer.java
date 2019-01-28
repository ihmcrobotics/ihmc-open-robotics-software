package us.ihmc.robotDataLogger.websocket.server;


import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.HttpServerCodec;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler;
import io.netty.handler.codec.http.websocketx.extensions.compression.WebSocketServerCompressionHandler;

public class WebsocketLogServerInitializer extends ChannelInitializer<SocketChannel>

{
   private static final String WEBSOCKET_PATH = "/websocket";

   private final WebsocketDataBroadcaster broadcaster;
   
   public WebsocketLogServerInitializer(WebsocketDataBroadcaster broadcaster)
   {
      this.broadcaster = broadcaster;
   }


   @Override
   protected void initChannel(SocketChannel ch) throws Exception
   {
      ChannelPipeline pipeline = ch.pipeline();

      pipeline.addLast(new HttpServerCodec());
      pipeline.addLast(new HttpObjectAggregator(65536));
      pipeline.addLast(new WebSocketServerCompressionHandler());
      pipeline.addLast(new WebSocketServerProtocolHandler(WEBSOCKET_PATH, null, true));
      pipeline.addLast(new WebsocketLogFrameHandler(broadcaster));
   }

}
