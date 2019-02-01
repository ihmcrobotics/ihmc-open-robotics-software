package us.ihmc.robotDataLogger.websocket.command;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.ByteBufUtil;
import io.netty.buffer.Unpooled;
import io.netty.util.CharsetUtil;

/**
 *
 * Commands available on the simple command server
 *
 * @author Jesper Smith
 *
 */
public enum DataServerCommands
{
   CLEAR_LOG(true),
   START_LOG(true),
   STOP_LOG(true),
   ;
   public static DataServerCommands[] values = values();
   
   private final ByteBuf content;
   
   DataServerCommands(boolean broadcast)
   {
      content = Unpooled.copiedBuffer(toString(), CharsetUtil.UTF_8);
   }
   
   public boolean startsWith(ByteBuf test)
   {
      if(test.readableBytes() < content.readableBytes())
      {
         return false;
      }
      
      
      for(int i = 0; i < content.readableBytes(); i++)
      {
         byte in = test.getByte(test.readerIndex() + i);
         byte orig = content.getByte(i);
         
         if(in != orig)
         {
            return false;
         }
      }
      
      return true;
      
   }
}
