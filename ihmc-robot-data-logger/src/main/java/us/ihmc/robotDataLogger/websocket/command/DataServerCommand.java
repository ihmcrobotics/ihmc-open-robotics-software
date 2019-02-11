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
   /**
    * Start sending timestamps to the client
    * 
    * Argument: Client port. Valid range [1025 - 65535]
    */
   SEND_TIMESTAMPS(false),
   
   /**
    * Limit the amount of packets to the rate argument
    * 
    * Set the rate argument
    * 
    * Argument: Rate in milliseconds. Valid range: [1 - 99999]ms
    */
   LIMIT_RATE(false),
   
   /**
    * Broadcast a clear log message. All loggers will restart their log session
    */
   CLEAR_LOG(true),
   /**   
    * Broadcast logger status
    * 
    * Argument: Length of log session in seconds  
    * 
    */
   LOG_ACTIVE(true),
   /**   
    * Broadcast logger status
    * 
    * Argument: Length of log session in seconds 
    * 
    */
   LOG_ACTIVE_WITH_CAMERA(true),
   /**
    * Broadcast a start log message. 
    * 
    * Not implemented
    */
   START_LOG(true), 
   /**
    * Broadcast a stop log message
    * 
    * Not implemented
    */
   STOP_LOG(true);
   

   private static final int MAX_ARGUMENT_SIZE = 5;

   public static DataServerCommand[] values = values();

   public static int MaxCommandSize()
   {
      int maxSize = 0;
      for (DataServerCommand cmd : values)
      {
         if (cmd.toString().length() > maxSize)
         {
            maxSize = cmd.length;
         }
      }

      return maxSize + MAX_ARGUMENT_SIZE;
   }

   public static DataServerCommand getCommand(ByteBuf in)
   {
      for (DataServerCommand cmd : values)
      {
         if (cmd.isThisCommand(in))
         {
            return cmd;
         }
      }
      return null;
   }

   private final ByteBuf content;
   private final int length;
   private final boolean broadcast;

   DataServerCommand(boolean broadcast)
   {
      this.content = Unpooled.copiedBuffer(toString(), CharsetUtil.UTF_8);
      this.length = content.readableBytes();
      this.broadcast = broadcast;
   }

   /**
    * @return true if the server should send this packet to all clients
    */
   public boolean broadcast()
   {
      return broadcast;
   }

   /**
    * Check if this buffer matches this command
    * 
    * @param test Test buffer, not changed
    * @return true if starts with this command
    */
   public boolean isThisCommand(ByteBuf test)
   {
      
      if(test.readableBytes() != content.readableBytes() && test.readableBytes() != content.readableBytes() + MAX_ARGUMENT_SIZE)
      {
         return false;
      }

      for (int i = 0; i < content.readableBytes(); i++)
      {
         byte in = test.getByte(test.readerIndex() + i);
         byte orig = content.getByte(i);

         if (in != orig)
         {
            return false;
         }
      }
      
      if(test.readableBytes() == content.readableBytes() + MAX_ARGUMENT_SIZE)
      {
         for(int i = content.readableBytes(); i < test.readableBytes(); i++)
         {
            byte in = test.getByte(test.readerIndex() + i);
            if(in < 48 || in > 57)
            {
               return false;
            }
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
      if (in.readableBytes() == length)
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
      if (argument < 0 || argument >= multiplier)
      {
         throw new RuntimeException("Invalid argument");
      }

      out.writeBytes(content, 0, content.readableBytes());

      for (int i = 0; i < MAX_ARGUMENT_SIZE; i++)
      {
         multiplier /= 10;
         int val = (argument / multiplier) % 10;

         out.writeByte(val + 48);

      }

   }
   
   public static int getMaximumArgumentValue()
   {
      return (int) Math.pow(10, MAX_ARGUMENT_SIZE) - 1;
   }

}
