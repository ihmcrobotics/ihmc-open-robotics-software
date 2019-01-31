package us.ihmc.robotDataLogger.websocket.server;

import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import io.netty.channel.Channel;
import io.netty.channel.ChannelFutureListener;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelPromise;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler.HandshakeComplete;

public class WebsocketDataServerFrameHandler extends SimpleChannelInboundHandler<WebSocketFrame>
{
   private static final int POOL_SIZE = 12;
   
   
   private final WebsocketDataBroadcaster broadcaster;
   private final int dataSize;
   
   private Object lock;
   private WriteTask task;
   private WebsocketFramePool pool;
   private Channel channel = null;
   private CustomGCAvoidingByteBufAllocator alloc = null;
   
   private final ArrayList<WebSocketFrame> queue = new ArrayList<WebSocketFrame>(POOL_SIZE);

   public WebsocketDataServerFrameHandler(WebsocketDataBroadcaster broadcaster, int dataSize)
   {
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
   }

   @Override
   public void userEventTriggered(ChannelHandlerContext ctx, Object evt) throws Exception
   {

      if (evt instanceof HandshakeComplete)
      {
         this.lock = new Object();
         this.pool = new WebsocketFramePool(dataSize, POOL_SIZE);
         this.task = new WriteTask();
         
         
         alloc = new CustomGCAvoidingByteBufAllocator(ctx.alloc());
         ctx.channel().config().setAllocator(alloc);
         
         channel = ctx.channel();
         broadcaster.addClient(this);
      }
      else
      {
         super.userEventTriggered(ctx, evt);
      }
   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, WebSocketFrame frame) throws Exception
   {
      if (frame instanceof TextWebSocketFrame)
      {
         String request = ((TextWebSocketFrame) frame).text();
         System.out.println(request);
      }
      else
      {
         String message = "unsupported frame type: " + frame.getClass().getName();
         throw new UnsupportedOperationException(message);
      }
   }

   @Override
   public void channelActive(ChannelHandlerContext ctx) throws Exception
   {
   }

   public void addCloseFutureListener(ChannelFutureListener listener)
   {
      channel.closeFuture().addListener(listener);
   }

   public SocketAddress remoteAddress()
   {
      return channel.remoteAddress();
   }
   
   Channel channel()
   {
      return channel;
   }

   public void write(ByteBuffer frame)
   {
      
      WebSocketFrame websocketFrame = pool.createFrame(frame);
      if(websocketFrame != null)
      {
         synchronized(lock)
         {
            queue.add(websocketFrame);            

            if(!task.scheduled)
            {
               task.init();
               channel.eventLoop().execute(task);
            }
         }
      }
   }

   private class WriteTask implements Runnable
   {
      private volatile boolean scheduled = false;
      private final ArrayList<WebSocketFrame> queueCopy = new ArrayList<>(POOL_SIZE);
      
      public void init()
      {
         this.scheduled = true;
      }

      @Override
      public void run()
      {
         ChannelPromise voidPromise = channel.voidPromise();
         if(channel.isActive() && channel.isWritable())
         {
            synchronized(lock)
            {
               for(int i = 0; i < queue.size(); i++)
               {
                  queueCopy.add(queue.get(i));
               }
               queue.clear();
            }
            
            for(int i = 0; i < queueCopy.size(); i++)
            {
               channel.write(queueCopy.get(i), voidPromise);
            }
            channel.flush();
            queueCopy.clear();
         }
         
         scheduled = false;
      }

   }

   public void release()
   {
      synchronized(lock)
      {
         if(channel.isActive() || channel.isWritable())
         {
            throw new RuntimeException("Trying to release an active channel");
         }
         
         if(alloc != null)
         {
            alloc.release();
         }
         
         if(pool != null)
         {
            pool.release();
         }
      }
   }
}
