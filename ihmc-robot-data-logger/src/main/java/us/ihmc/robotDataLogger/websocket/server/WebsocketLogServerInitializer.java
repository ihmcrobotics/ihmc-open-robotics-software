package us.ihmc.robotDataLogger.websocket.server;


import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.HttpServerCodec;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler;

public class WebsocketLogServerInitializer extends ChannelInitializer<SocketChannel>

{
   private static final String WEBSOCKET_PATH = "/websocket";

   private final LogServerContent logServerContent;
   private final WebsocketDataBroadcaster broadcaster;
   private final int dataSize;
   
   public WebsocketLogServerInitializer(LogServerContent logServerContent, WebsocketDataBroadcaster broadcaster, int dataSize)
   {
      this.logServerContent = logServerContent;
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
   }


   @Override
   protected void initChannel(SocketChannel ch) throws Exception
   {
      ChannelPipeline pipeline = ch.pipeline();

      pipeline.addLast(new HttpServerCodec());
      pipeline.addLast(new HttpObjectAggregator(65536));
      pipeline.addLast(new WebSocketServerProtocolHandler(WEBSOCKET_PATH, null, true));
      pipeline.addLast(new WebsocketLogDescriptionServer(logServerContent));
      pipeline.addLast(new WebsocketLogFrameHandler(broadcaster, dataSize));
   }

}
