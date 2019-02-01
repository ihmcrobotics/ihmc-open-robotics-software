package us.ihmc.robotDataLogger.websocket.command;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.util.CharsetUtil;

/**
 *
 * Commands available on the simple command server
 *
 * @author Jesper Smith
 *
 */
public enum DataServerCommand
{
   CLEAR_LOG(true),
   START_LOG(true),
   STOP_LOG(true),
   SEND_TIMESTAMPS(false)
   ;
 
   private static final int MAX_ARGUMENT_SIZE = 5;
   
   public static DataServerCommand[] values = values();
   
   public static int MaxCommandSize()
   {
      int maxSize = 0;
      for(DataServerCommand cmd : values)
      {
         if(cmd.toString().length() > maxSize)
         {
            maxSize = cmd.length;
         }
      }
      
      return maxSize + MAX_ARGUMENT_SIZE;
   }
   
   private final ByteBuf content;
   private final int length;
   
   
   DataServerCommand(boolean broadcast)
   {
      content = Unpooled.copiedBuffer(toString(), CharsetUtil.UTF_8);
      length = content.readableBytes();
   }
   
   /**
    * Check if this buffer matches this command
    * 
    * @param test Test buffer, not changed
    * @return true if starts with this command
    */
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
   
   /**
    * Get the argument for this call
    * 
    * @param in Input buffer. Not changed
    * @return -1 if buffer is not long enough, 0 if buffer length equals length of command, argument otherwise
    */
   public int getArgument(ByteBuf in)
   {
      if(in.readableBytes() == length)
      {
         return 0;
      }
      else if (in.readableBytes() == length + MAX_ARGUMENT_SIZE)
      {
         int multiplier = (int) Math.pow(10, MAX_ARGUMENT_SIZE);

         int argument = 0;
         for (int i = 0; i < MAX_ARGUMENT_SIZE; i++)
         {
            multiplier /= 10;
            argument += (in.getByte(length + i) - 48) * multiplier;
         }
         return argument;
      }
      else
      {
         return -1;
      }
   }
   
   public void getBytes(ByteBuf out)
   {
      out.writeBytes(content, 0, content.readableBytes());
   }
   
   public void getBytes(ByteBuf out, int argument)
   {
      int multiplier = (int) Math.pow(10, MAX_ARGUMENT_SIZE);
      if(argument < 0 || argument >= multiplier)
      {
         throw new RuntimeException("Invalid argument");
      }
      
      out.writeBytes(content, 0, content.readableBytes());
      
      
      for(int i = 0; i < MAX_ARGUMENT_SIZE; i++)
      {
         multiplier /= 10;
         int val = (argument / multiplier) % 10;
                  
         out.writeByte(val + 48);
         
      }
   }
   
}
