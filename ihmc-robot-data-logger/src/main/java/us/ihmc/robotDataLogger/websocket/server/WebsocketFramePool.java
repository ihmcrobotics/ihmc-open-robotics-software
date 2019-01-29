package us.ihmc.robotDataLogger.websocket.server;

import java.nio.ByteBuffer;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;

public class WebsocketFramePool
{
   private static class WebsocketFrameAndBuffer
   {

      private final ByteBuf buffer;
      private final BinaryWebSocketFrame frame;

      private WebsocketFrameAndBuffer(int bufferSize)
      {
         buffer = Unpooled.directBuffer(bufferSize, bufferSize);
         frame = new BinaryWebSocketFrame(buffer);
      }
   }

   private int index = 0;
   private final WebsocketFrameAndBuffer pool[];

   public WebsocketFramePool(int bufferSize, int poolSize)
   {
      pool = new WebsocketFrameAndBuffer[poolSize];
      for (int i = 0; i < poolSize; i++)
      {
         pool[i] = new WebsocketFrameAndBuffer(bufferSize);
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
}
