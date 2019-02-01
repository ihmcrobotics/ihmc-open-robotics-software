package us.ihmc.robotDataLogger.websocket.server;

import java.nio.ByteBuffer;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;

/**
 * 
 * Pooled buffer of BinaryWebsocketFrames and ByteBuf to avoid object allocation on transmit.
 * 
 * When a ByteBuf is requested, the refcnt is increased using retain(). When the refcnt is reset to 1, it is re-usable
 * 
 * If the buffer is full, no element is returned and the transmission should be skipped.
 * 
 * @author Jesper Smith
 *
 */
class WebsocketFramePool
{
   private static class WebsocketFrameAndBuffer
   {

      private final ByteBuf buffer;
      private final WebSocketFrame frame;

      private WebsocketFrameAndBuffer(int bufferSize, Class<? extends WebSocketFrame> type)
      {
         buffer = Unpooled.directBuffer(bufferSize, bufferSize);
         if(type == BinaryWebSocketFrame.class)
         {
            frame = new BinaryWebSocketFrame(buffer);            
         }
         else if(type == TextWebSocketFrame.class)
         {
            frame = new TextWebSocketFrame(buffer);
         }
         else
         {
            throw new RuntimeException("Invalid type");
         }
      }
   }

   private int index = 0;
   private final WebsocketFrameAndBuffer pool[];

   public WebsocketFramePool(int bufferSize, int poolSize, Class<? extends WebSocketFrame> type)
   {
      pool = new WebsocketFrameAndBuffer[poolSize];
      for (int i = 0; i < poolSize; i++)
      {
         pool[i] = new WebsocketFrameAndBuffer(bufferSize, type);
      }
   }
   
   private WebsocketFrameAndBuffer getNext()
   {
      
      for(int i = index; i < index + pool.length; i++)
      {
         int item = i < pool.length ? i : i - pool.length;
         if(pool[item].buffer.refCnt() == 1)
         {
            pool[item].buffer.retain();
            index = item + 1;
            return pool[item];
         }
      }
      
      return null;
   }
   
   
   public WebSocketFrame createFrame()
   {
      WebsocketFrameAndBuffer next = getNext();
      if(next != null)
      {
         next.buffer.clear();
         return next.frame;
      }
      else
      {
         return null;
      }
   }
   
   public WebSocketFrame createFrame(ByteBuffer data)
   {
      WebsocketFrameAndBuffer next = getNext();
      if(next != null)
      {
         next.buffer.clear();
         next.buffer.writeBytes(data);
         data.flip();
         return next.frame;
      }
      else
      {
         return null;
      }
   }
   
   public void release()
   {
      for(int i = 0; i < pool.length; i++)
      {
         pool[i].buffer.release();
      }
   }
}
