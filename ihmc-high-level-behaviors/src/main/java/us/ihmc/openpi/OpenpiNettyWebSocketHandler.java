package us.ihmc.openpi;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelPromise;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.DefaultHttpHeaders;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.CloseWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshaker;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshakerFactory;
import io.netty.handler.codec.http.websocketx.WebSocketHandshakeException;
import io.netty.handler.codec.http.websocketx.WebSocketVersion;
import us.ihmc.log.LogTools;

import java.net.URI;
import java.util.Queue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

class OpenpiNettyWebSocketHandler extends SimpleChannelInboundHandler<Object>
{
   private final URI uri;
   private final WebSocketClientHandshaker handshaker;
   private ChannelPromise handshakeFuture;

   private final Queue<CompletableFuture<byte[]>> pendingResponses = new ConcurrentLinkedQueue<>();
   private final CountDownLatch firstMessageLatch = new CountDownLatch(1);
   private byte[] firstMessage;
   private volatile RuntimeException firstMessageFailure;

   public OpenpiNettyWebSocketHandler(URI uri)
   {
      this.handshaker = WebSocketClientHandshakerFactory.newHandshaker(uri, WebSocketVersion.V13, null, false, new DefaultHttpHeaders());
      this.uri = uri;
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
   protected void channelRead0(ChannelHandlerContext ctx, Object msg)
   {
      Channel ch = ctx.channel();

      if (!handshaker.isHandshakeComplete())
      {
         try
         {
            handshaker.finishHandshake(ch, (FullHttpResponse) msg);
            LogTools.info("Connected to openpi server at " + uri);
            handshakeFuture.setSuccess();
         }
         catch (WebSocketHandshakeException e)
         {
            LogTools.error("WebSocket handshake failed");
            handshakeFuture.setFailure(e);
         }
         return;
      }

      if (msg instanceof FullHttpResponse response)
      {
         throw new IllegalStateException("Unexpected FullHttpResponse (getStatus=" + response.status() + ", content=" + response.content().toString() + ')');
      }

      if (msg instanceof TextWebSocketFrame textFrame)
      {
         String errorMsg = textFrame.text();
         LogTools.error("Error from server: " + errorMsg);
         RuntimeException failure = new RuntimeException("Server error: " + errorMsg);

         if (firstMessage == null)
            failFirstMessage(failure);
         else
            completeNextResponseExceptionally(failure);
      }
      else if (msg instanceof BinaryWebSocketFrame binaryFrame)
      {
         ByteBuf content = binaryFrame.content();
         byte[] data = new byte[content.readableBytes()];
         content.readBytes(data);

         // Handle first message (server metadata)
         if (firstMessage == null)
         {
            firstMessage = data;
            firstMessageLatch.countDown();
         }
         else
         {
            // Handle response to inference request
            CompletableFuture<byte[]> pending = pendingResponses.poll();
            if (pending != null)
            {
               pending.complete(data);
            }
         }
      }
      else if (msg instanceof CloseWebSocketFrame)
      {
         LogTools.warn("WebSocket connection closed");
         failConnection(new RuntimeException("WebSocket connection closed before the server responded"));
         ch.close();
      }
   }

   @Override
   public void channelInactive(ChannelHandlerContext ctx)
   {
      failConnection(new RuntimeException("WebSocket connection became inactive before the server responded"));
      ctx.fireChannelInactive();
   }

   @Override
   public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause)
   {
      LogTools.error("OpenPI websocket failed: " + cause.getMessage());
      if (handshakeFuture != null && !handshakeFuture.isDone())
      {
         handshakeFuture.setFailure(cause);
      }
      failConnection(new RuntimeException("OpenPI websocket failed", cause));
      ctx.close();
   }

   private void completeNextResponseExceptionally(RuntimeException failure)
   {
      CompletableFuture<byte[]> pending = pendingResponses.poll();
      if (pending != null)
         pending.completeExceptionally(failure);
   }

   private void failFirstMessage(RuntimeException failure)
   {
      if (firstMessage == null && firstMessageFailure == null)
      {
         firstMessageFailure = failure;
         firstMessageLatch.countDown();
      }
   }

   private void failConnection(RuntimeException failure)
   {
      failFirstMessage(failure);
      CompletableFuture<byte[]> pending;
      while ((pending = pendingResponses.poll()) != null)
         pending.completeExceptionally(failure);
   }

   public CompletableFuture<byte[]> sendAndAwaitResponse(byte[] data)
   {
      CompletableFuture<byte[]> future = new CompletableFuture<>();
      pendingResponses.offer(future);

      ByteBuf buffer = Unpooled.wrappedBuffer(data);
      BinaryWebSocketFrame frame = new BinaryWebSocketFrame(buffer);

      handshakeFuture.channel().writeAndFlush(frame).addListener(channelFuture ->
      {
         if (!channelFuture.isSuccess())
         {
            pendingResponses.remove(future);
            future.completeExceptionally(channelFuture.cause());
         }
      });

      return future;
   }

   public byte[] awaitFirstMessage(long timeout, TimeUnit unit) throws Exception
   {
      if (firstMessageLatch.await(timeout, unit))
      {
         if (firstMessageFailure != null)
            throw firstMessageFailure;
         return firstMessage;
      }
      else
      {
         throw new RuntimeException("Timeout waiting for server metadata");
      }
   }
}
