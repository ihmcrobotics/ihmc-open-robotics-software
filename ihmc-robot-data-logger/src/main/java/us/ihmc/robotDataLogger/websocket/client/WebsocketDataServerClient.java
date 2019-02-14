package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
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
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshakerFactory;
import io.netty.handler.codec.http.websocketx.WebSocketVersion;
import io.netty.handler.codec.http.websocketx.extensions.compression.WebSocketClientCompressionHandler;
import io.netty.handler.timeout.IdleStateHandler;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.dataBuffers.CustomLogDataSubscriberType;
import us.ihmc.robotDataLogger.dataBuffers.RegistryConsumer;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection.DisconnectPromise;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

public class WebsocketDataServerClient
{
   private final EventLoopGroup group = new NioEventLoopGroup();
   private final RegistryConsumer consumer;

   private final VariableChangeRequestPubSubType variableChangeRequestType = new VariableChangeRequestPubSubType();
   private final SerializedPayload variableChangeRequestPayload = new SerializedPayload(variableChangeRequestType.getTypeSize());

   private final Channel ch;

   private final DisconnectPromise disconnectPromise;
   private final UDPTimestampClient udpTimestampClient;

   public WebsocketDataServerClient(HTTPDataServerConnection connection, IDLYoVariableHandshakeParser parser, TimestampListener timestampListener,
                                    YoVariableClientImplementation yoVariableClient, int timeoutInMs, DebugRegistry debugRegistry)
         throws IOException
   {
      this.disconnectPromise = connection.take();
      HTTPDataServerDescription target = connection.getTarget();

      URI uri;
      try
      {
         uri = new URI("ws://" + target.getHost() + ":" + target.getPort() + "/websocket");
      }
      catch (URISyntaxException e)
      {
         throw new IOException(e);
      }

      this.consumer = new RegistryConsumer(parser, yoVariableClient, debugRegistry);
      this.udpTimestampClient = new UDPTimestampClient(timestampListener);
      this.udpTimestampClient.start();


      CustomLogDataSubscriberType type = new CustomLogDataSubscriberType(parser.getNumberOfVariables(), parser.getNumberOfStates());
      final WebSocketDataServerClientHandler handler = new WebSocketDataServerClientHandler(WebSocketClientHandshakerFactory.newHandshaker(uri,
                                                                                                                                           WebSocketVersion.V13,
                                                                                                                                           null, true,
                                                                                                                                           new DefaultHttpHeaders()),
                                                                                            yoVariableClient, udpTimestampClient.getPort(), consumer, type);

      Bootstrap b = new Bootstrap();
      b.group(group).channel(NioSocketChannel.class).handler(new ChannelInitializer<SocketChannel>()
      {
         @Override
         protected void initChannel(SocketChannel ch)
         {
            ChannelPipeline p = ch.pipeline();
            p.addLast(new HttpClientCodec(), new HttpObjectAggregator(65536), WebSocketClientCompressionHandler.INSTANCE, new IdleStateHandler(timeoutInMs, 0, 0, TimeUnit.MILLISECONDS), handler);
         }
      });

      b.option(ChannelOption.CONNECT_TIMEOUT_MILLIS, 1000);

      try
      {
         ch = b.connect(uri.getHost(), uri.getPort()).syncUninterruptibly().channel();
         ch.closeFuture().addListener((e) -> disconnected());
      }
      catch (Exception e)
      {
         disconnected();
         throw new IOException(e);
      }

   }

   private void disconnected()
   {
      this.udpTimestampClient.stop();
      this.udpTimestampClient.join();
      consumer.stopImmediatly();
      try
      {
         consumer.join();
      }
      catch (InterruptedException e)
      {
      }
      disconnectPromise.complete();
      group.shutdownGracefully();
   }

   public boolean isActive()
   {
      return ch.isActive();
   }

   public void close()
   {
      ch.close();
   }

   public void writeVariableChangeRequest(int identifier, double valueAsDouble)
   {
      try
      {
         VariableChangeRequest msg = new VariableChangeRequest();
         msg.setVariableID(identifier);
         msg.setRequestedValue(valueAsDouble);
         
         variableChangeRequestPayload.getData().clear();
         variableChangeRequestType.serialize(msg, variableChangeRequestPayload);

         ByteBuf data = ch.alloc().buffer(variableChangeRequestPayload.getLength());
         data.writeBytes(variableChangeRequestPayload.getData());
         BinaryWebSocketFrame frame = new BinaryWebSocketFrame(data);
         ch.writeAndFlush(frame);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void sendCommand(DataServerCommand command, int argument)
   {
      try
      {
         ByteBuf cmdBuf = ch.alloc().buffer(DataServerCommand.MaxCommandSize());
         command.getBytes(cmdBuf, argument);
         TextWebSocketFrame frame = new TextWebSocketFrame(cmdBuf);

         ch.writeAndFlush(frame);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

}
