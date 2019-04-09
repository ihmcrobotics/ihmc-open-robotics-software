package us.ihmc.robotDataLogger.websocket.server;


import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.HttpServerCodec;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.LogAliveListener;

/**
 * Initializes a connection
 * 
 * @author Jesper Smith
 *
 */
class WebsocketDataServerInitializer extends ChannelInitializer<SocketChannel>
{
   private static final String WEBSOCKET_PATH = "/websocket";

   private final DataServerServerContent logServerContent;
   private final WebsocketDataBroadcaster broadcaster;
   private final VariableChangedListener variableChangedListener;
   private final LogAliveListener logAliveListener;
   private final int dataSize;
   private final int numberOfRegistryBuffers;

   public WebsocketDataServerInitializer(DataServerServerContent logServerContent, WebsocketDataBroadcaster broadcaster,
                                         VariableChangedListener variableChangedListener, LogAliveListener logAliveListener, int dataSize,
                                         int numberOfRegistryBuffers)
   {
      this.logServerContent = logServerContent;
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
      this.variableChangedListener = variableChangedListener;
      this.logAliveListener = logAliveListener;
      this.numberOfRegistryBuffers = numberOfRegistryBuffers;
   }


   @Override
   protected void initChannel(SocketChannel ch) throws Exception
   {
      ChannelPipeline pipeline = ch.pipeline();

      pipeline.addLast(new DataServerIOExceptionHandler());
      pipeline.addLast(new HttpServerCodec());
      pipeline.addLast(new HttpObjectAggregator(65536));
      pipeline.addLast(new WebSocketServerProtocolHandler(WEBSOCKET_PATH, null, true));
      pipeline.addLast(new HTTPDataServerDescriptionServer(logServerContent));
      pipeline.addLast(new WebsocketDataServerFrameHandler(broadcaster, dataSize, numberOfRegistryBuffers, variableChangedListener, logAliveListener));
   }

}
