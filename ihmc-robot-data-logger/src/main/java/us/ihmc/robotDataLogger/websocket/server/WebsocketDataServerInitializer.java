package us.ihmc.robotDataLogger.websocket.server;


import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.HttpServerCodec;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;

public class WebsocketDataServerInitializer extends ChannelInitializer<SocketChannel>

{
   private static final String WEBSOCKET_PATH = "/websocket";

   private final DataServerServerContent logServerContent;
   private final WebsocketDataBroadcaster broadcaster;
   private final VariableChangedListener variableChangedListener;
   private final int dataSize;
   
   public WebsocketDataServerInitializer(DataServerServerContent logServerContent, WebsocketDataBroadcaster broadcaster, VariableChangedListener variableChangedListener, int dataSize)
   {
      this.logServerContent = logServerContent;
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
      this.variableChangedListener = variableChangedListener;
   }


   @Override
   protected void initChannel(SocketChannel ch) throws Exception
   {
      ChannelPipeline pipeline = ch.pipeline();

      pipeline.addLast(new HttpServerCodec());
      pipeline.addLast(new HttpObjectAggregator(65536));
      pipeline.addLast(new WebSocketServerProtocolHandler(WEBSOCKET_PATH, null, true));
      pipeline.addLast(new HTTPDataServerDescriptionServer(logServerContent));
      pipeline.addLast(new WebsocketDataServerFrameHandler(broadcaster, dataSize, variableChangedListener));
   }

}
