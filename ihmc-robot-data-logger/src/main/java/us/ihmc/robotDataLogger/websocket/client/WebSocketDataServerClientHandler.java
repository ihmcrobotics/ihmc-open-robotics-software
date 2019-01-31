package us.ihmc.robotDataLogger.websocket.client;

import io.netty.buffer.ByteBuf;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelPromise;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.CloseWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PongWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshaker;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketHandshakeException;
import io.netty.util.CharsetUtil;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.dataBuffers.RegistryConsumer;
import us.ihmc.robotDataLogger.dataBuffers.RegistryReceiveBuffer;
import us.ihmc.robotDataLogger.rtps.CustomLogDataSubscriberType;

public class WebSocketDataServerClientHandler extends SimpleChannelInboundHandler<Object>
{

   private final WebSocketClientHandshaker handshaker;
   private final RegistryConsumer consumer;
   private final YoVariableClientImplementation yoVariableClient;

   private final CustomLogDataSubscriberType type;
   private final SerializedPayload payload;
   
   private ChannelPromise handshakeFuture;

   public WebSocketDataServerClientHandler(WebSocketClientHandshaker handshaker, YoVariableClientImplementation yoVariableClient, RegistryConsumer consumer, CustomLogDataSubscriberType type)
   {
      this.handshaker = handshaker;
      this.yoVariableClient = yoVariableClient;
      this.consumer = consumer;
      this.type = type;
      
      this.payload = new SerializedPayload(type.getTypeSize());
   }

   public ChannelFuture handshakeFuture()
   {
      return handshakeFuture;
   }

   @Override
   public void handlerAdded(ChannelHandlerContext ctx)
   {
      handshakeFuture = ctx.newPromise();
   }

   @Override
   public void channelActive(ChannelHandlerContext ctx)
   {
      handshaker.handshake(ctx.channel());
   }

   @Override
   public void channelInactive(ChannelHandlerContext ctx)
   {
      consumer.stopImmediatly();
   }

   @Override
   public void channelRead0(ChannelHandlerContext ctx, Object msg) throws Exception
   {
      Channel ch = ctx.channel();
      if (!handshaker.isHandshakeComplete())
      {
         try
         {
            handshaker.finishHandshake(ch, (FullHttpResponse) msg);
            yoVariableClient.connected();
            handshakeFuture.setSuccess();
         }
         catch (WebSocketHandshakeException e)
         {
            e.printStackTrace();
            consumer.stopImmediatly();
            handshakeFuture.setFailure(e);
         }
         return;
      }

      if (msg instanceof FullHttpResponse)
      {
         FullHttpResponse response = (FullHttpResponse) msg;
         throw new IllegalStateException("Unexpected FullHttpResponse (getStatus=" + response.status() + ", content="
               + response.content().toString(CharsetUtil.UTF_8) + ')');
      }

      WebSocketFrame frame = (WebSocketFrame) msg;
      if (frame instanceof TextWebSocketFrame)
      {
         // Discard
      }
      else if (frame instanceof BinaryWebSocketFrame)
      {
         RegistryReceiveBuffer buffer = new RegistryReceiveBuffer(System.nanoTime());
         payload.getData().clear();
         payload.getData().limit(frame.content().readableBytes());
         frame.content().readBytes(payload.getData());
         payload.getData().flip();
         type.deserialize(payload, buffer);
         consumer.onNewDataMessage(buffer);
      }
      else if (frame instanceof PongWebSocketFrame)
      {
         System.out.println("WebSocket Client received pong");
      }
      else if (frame instanceof CloseWebSocketFrame)
      {
         System.out.println("WebSocket Client received closing");
         ch.close();
      }
   }

   @Override
   public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause)
   {
      cause.printStackTrace();
      if (!handshakeFuture.isDone())
      {
         handshakeFuture.setFailure(cause);
      }
      ctx.close();
   }
}