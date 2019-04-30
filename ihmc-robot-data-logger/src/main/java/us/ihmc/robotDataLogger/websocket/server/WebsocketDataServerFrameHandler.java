package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

import io.netty.channel.Channel;
import io.netty.channel.ChannelFutureListener;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelPromise;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PingWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PongWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler.HandshakeComplete;
import us.ihmc.commons.Conversions;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.VariableChangeRequest;
import us.ihmc.robotDataLogger.VariableChangeRequestPubSubType;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

/**
 * Handler for websocket connection
 * 
 * - Handles writing data to channel
 * - Passes incoming variableChangeRequests to the variablechangedlistener
 * - Handles the simple command server 
 * 
 * @author Jesper Smith
 *
 */
class WebsocketDataServerFrameHandler extends SimpleChannelInboundHandler<WebSocketFrame>
{
   private static final int BINARY_POOL_SIZE = 12;
   private static final int TEXT_POOL_SIZE = 128;

   private final WebsocketDataBroadcaster broadcaster;
   private final VariableChangedListener variableChangedListener;
   private final LogAliveListener logAliveListener;
   private final int dataSize;

   private Object lock;
   private WebsocketFramePool binaryPool;
   private WebsocketFramePool textPool = new WebsocketFramePool(DataServerCommand.MaxCommandSize(), TEXT_POOL_SIZE, TextWebSocketFrame.class);
   private Channel channel = null;
   private RecyclingByteBufAllocator alloc = null;


   private final VariableChangeRequestPubSubType variableChangeRequestType = new VariableChangeRequestPubSubType();
   private final SerializedPayload variableChangeRequestPayload = new SerializedPayload(variableChangeRequestType.getTypeSize());
   private final VariableChangeRequest request = new VariableChangeRequest();

   private final UDPTimestampServer udpTimestampServer;
   
   
   private final WebsocketDataServerRegistrySendStatistics[] registryStatistics;
   
   private long requestedUpdateDT = 0;

   public WebsocketDataServerFrameHandler(WebsocketDataBroadcaster broadcaster, int dataSize, int numberOfRegistryBuffers, VariableChangedListener variableChangedListener,
                                          LogAliveListener logAliveListener)
         throws IOException
   {
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
      this.variableChangedListener = variableChangedListener;
      this.logAliveListener = logAliveListener;
      this.udpTimestampServer = new UDPTimestampServer();
      
      registryStatistics = new WebsocketDataServerRegistrySendStatistics[numberOfRegistryBuffers];
      for(int i = 0; i < numberOfRegistryBuffers; i++)
      {
         registryStatistics[i] = new WebsocketDataServerRegistrySendStatistics();
      }
   }

   @Override
   public void userEventTriggered(ChannelHandlerContext ctx, Object evt) throws Exception
   {

      if (evt instanceof HandshakeComplete)
      {
         this.lock = new Object();
         this.binaryPool = new WebsocketFramePool(dataSize, BINARY_POOL_SIZE, BinaryWebSocketFrame.class);

         alloc = new RecyclingByteBufAllocator(ctx.alloc());
         ctx.channel().config().setAllocator(alloc);

         channel = ctx.channel();
         broadcaster.addClient(this);
      }
      else
      {
         super.userEventTriggered(ctx, evt);
      }
   }

   private void runCommand(DataServerCommand command, int argument)
   {
      switch(command)
      {
      case SEND_TIMESTAMPS:
         udpTimestampServer.startSending(remoteAddress().getAddress(), argument);
         break;
      case LIMIT_RATE:
         synchronized(lock)
         {
            requestedUpdateDT = Conversions.millisecondsToNanoseconds(argument);
         }
         break;
      case LOG_ACTIVE:
         if (logAliveListener != null)
         {
            logAliveListener.receivedLogAliveCommand(false);
         }
         break;
      case LOG_ACTIVE_WITH_CAMERA:
         if (logAliveListener != null)
         {
            logAliveListener.receivedLogAliveCommand(true);
         }
         break;
         default:
            // Do nothing
      }
      

      if (command.broadcast())
      {
         broadcaster.writeCommand(command, argument);
      }

   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, WebSocketFrame frame) throws Exception
   {
      try
      {
         if (frame instanceof TextWebSocketFrame)
         {
            DataServerCommand command = DataServerCommand.getCommand(frame.content());
            if (command != null)
            {
               int argument = command.getArgument(frame.content());
               if (argument != -1)
               {
                  runCommand(command, argument);
               }
            }
         }
         else if (frame instanceof BinaryWebSocketFrame)
         {
            variableChangeRequestPayload.getData().clear();
            variableChangeRequestPayload.getData().limit(frame.content().readableBytes());
            frame.content().readBytes(variableChangeRequestPayload.getData());
            variableChangeRequestPayload.getData().flip();
            variableChangeRequestType.deserialize(variableChangeRequestPayload, request);
            variableChangedListener.changeVariable(request.getVariableID(), request.getRequestedValue());

         }
         else if (frame instanceof PingWebSocketFrame)
         {
         }
         else if (frame instanceof PongWebSocketFrame)
         {
            
         }
         else
         {
            String message = "unsupported frame type: " + frame.getClass().getName();
            throw new UnsupportedOperationException(message);
         }
      }
      catch (Exception e)
      {
         // Swallow exceptions to avoid object allocations
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

   public InetSocketAddress remoteAddress()
   {
      return (InetSocketAddress) channel.remoteAddress();
   }

   Channel channel()
   {
      return channel;
   }
      
   
   /**
    * Updates the registry count and estimated dt for bufferD 
    * 
    * @param bufferID
    */
   private void updateRegistryStatistics(int bufferID, long timestamp)
   {
      if(bufferID >= registryStatistics.length)
      {
         throw new RuntimeException("Invalid registry ID");  
      }
      
      registryStatistics[bufferID].update(timestamp);
      
   }
   
   /**
    * Check if an update should be send
    * 
    * An update should be send if enough time passed for the current registry buffer OR another registry buffer should send at this time.
    * 
    * Note: If timestamps are not increasing for any thread and no DT can be determined, no data will be send to the client.
    * 
    * @param bufferID
    * @param timestamp
    * @return true if we should send this registry
    */
   private boolean shouldSend(int bufferID, long timestamp)
   {
      // Always send if requestedUpdateDT is set to 0
      if(requestedUpdateDT == 0)
      {
         return true;
      }
      
      
      // Figure out what rate the fastest buffer updates at
      long fastestRegistryBufferDT = Long.MAX_VALUE;
      for(int i = 0; i < registryStatistics.length; i++) 
      {
         if(!registryStatistics[i].isNonMonotonic())
         {
            long dt = registryStatistics[i].getRegistryBufferDT();
            if(dt < fastestRegistryBufferDT)
            {
               fastestRegistryBufferDT = dt;
            }
         }
      }
            
      if(registryStatistics[bufferID].shouldSend(timestamp, requestedUpdateDT, fastestRegistryBufferDT, false))
      {
         return true;
      }
      else if (fastestRegistryBufferDT != Long.MAX_VALUE)      // Check if we need to send to match any buffer slower than the current buffer, but only if we have at least one monotonic updating buffer
      {
         for(int i = 0; i < registryStatistics.length; i++)
         {
            if(registryStatistics[i].getRegistryBufferDT() > registryStatistics[bufferID].getRegistryBufferDT())
            {
               if(registryStatistics[i].shouldSend(timestamp, requestedUpdateDT, fastestRegistryBufferDT, true))
               {
                  return true;
               }
            }
         }
      }
      
      return false;
      
   }
   
   private void updateRegistrySendTimestamp(int bufferID, long timestamp)
   {
      registryStatistics[bufferID].updateSendTimestamp(timestamp);
   }
   

   /**
    * Write binary data in "frame"
    * 
    * If called from the channel outbound event loop, no objects will be allocated.
    * @param bufferID 
    * 
    * @param frame
    */
   public void write(int bufferID, long timestamp, ByteBuffer frame)
   {
      if (!channel.eventLoop().inEventLoop())
      {
         throw new RuntimeException("Call this function from the channels event loop");
      }
      synchronized (lock)
      {
         updateRegistryStatistics(bufferID, timestamp);

         if (shouldSend(bufferID, timestamp))
         {
            WebSocketFrame websocketFrame = binaryPool.createFrame(frame);
            if (websocketFrame != null)
            {
               if (channel.isActive() && channel.isWritable())
               {
                  ChannelPromise voidPromise = channel.voidPromise();
                  channel.writeAndFlush(websocketFrame, voidPromise);
               }
            }
            
            updateRegistrySendTimestamp(bufferID, timestamp);
         }
      }
   }

   public void release()
   {
      synchronized (lock)
      {
         if (channel.isActive() || channel.isWritable())
         {
            throw new RuntimeException("Trying to release an active channel");
         }

         udpTimestampServer.close();

         if (alloc != null)
         {
            alloc.release();
         }

         if (binaryPool != null)
         {
            binaryPool.release();
         }
         
         if (textPool != null)
         {
            textPool.release();
         }
      }
   }

   public void writeCommand(DataServerCommand command, int argument)
   {

      if(!channel.eventLoop().inEventLoop())
      {
         throw new RuntimeException("Call this function from the channels event loop");
      }
      synchronized (lock)
      {
         WebSocketFrame websocketFrame = textPool.createFrame();
         if (websocketFrame != null)
         {
            if (channel.isActive()) // Do not check if the channel is writable. We want the commands to go out.
            {
               command.getBytes(websocketFrame.content(), argument);
               channel.writeAndFlush(websocketFrame);
            }

         }
      }
   }

   public void publishTimestamp(long timestamp)
   {
      udpTimestampServer.sendTimestamp(timestamp);
   }
}
