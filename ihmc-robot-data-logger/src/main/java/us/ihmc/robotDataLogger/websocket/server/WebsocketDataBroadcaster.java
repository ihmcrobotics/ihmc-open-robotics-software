package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.locks.LockSupport;

import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelFutureListener;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

/**
 * Helper class that keep track of active connections and writes data to all connections.
 * 
 * It also has an internal thread for the timestamp publisher to avoid the overhead of having a thread per connection for timestamp publishing
 * 
 * @author Jesper Smith
 *
 */
class WebsocketDataBroadcaster implements ChannelFutureListener
{
   /**
    * How much time the timestamp publishing thread should wait between successive polling for a new timestamp.
    */
   public static final int TIMESTAMP_PUBLISHING_SLEEP_NS = 500000;  
   
   
   private final Object channelLock = new Object();
   private final TimestampPublishingThread timestampPublishingThread = new TimestampPublishingThread();

   // Implement channels as copy on write array to avoid blocking in the timestamp update thread
   private volatile WebsocketDataServerFrameHandler[] channels = new WebsocketDataServerFrameHandler[0];

   private volatile boolean active = true;
   private volatile long newTimestamp = Long.MIN_VALUE;

   public WebsocketDataBroadcaster()
   {
      timestampPublishingThread.start();
   }

   public void addClient(WebsocketDataServerFrameHandler websocketLogFrameHandler)
   {

      synchronized (channelLock)
      {

         WebsocketDataServerFrameHandler[] newChannels = Arrays.copyOf(channels, channels.length + 1);
         newChannels[newChannels.length - 1] = websocketLogFrameHandler;
         channels = newChannels;

         websocketLogFrameHandler.addCloseFutureListener(this);
      }
   }

   public void write(int bufferID, long timestamp, ByteBuffer frame) throws IOException
   {
      // Localize channels
      WebsocketDataServerFrameHandler[] localChannels = channels;

      for (int i = 0; i < localChannels.length; i++)
      {
         localChannels[i].write(bufferID, timestamp, frame);
      }
   }

   /**
    * Remove channel on completion
    */
   @Override
   public void operationComplete(ChannelFuture future) throws Exception
   {
      synchronized (channelLock)
      {
         Channel channel = future.channel();
         WebsocketDataServerFrameHandler[] newChannels = new WebsocketDataServerFrameHandler[channels.length - 1];

         int newI = 0;
         for (int i = 0; i < channels.length; i++)
         {
            if (channels[i].channel() == channel)
            {
               channels[i].release();
            }
            else
            {
               if (newI >= newChannels.length)
               {
                  // Channel not found, returning
                  return;
               }

               newChannels[newI] = channels[i];
               ++newI;
            }
         }
         
         channels = newChannels;
      }
   }

   public void writeCommand(DataServerCommand command, int argument)
   {
      // Localize channels
      WebsocketDataServerFrameHandler[] localChannels = channels;

      for (int i = 0; i < localChannels.length; i++)
      {
         localChannels[i].writeCommand(command, argument);
      }

   }

   public void publishTimestamp(long timestamp)
   {
      newTimestamp = timestamp;

   }

   public void stop()
   {
      active = false;
   }

   /**
    * Internal thread that sends timestamps over a UDP connection.
    * 
    * Polls for new timestamps at approximately 10kHz
    * 
    * @author Jesper Smith
    *
    */
   private class TimestampPublishingThread extends Thread
   {

      private TimestampPublishingThread()
      {
         super(TimestampPublishingThread.class.getSimpleName());
      }

      @Override
      public void run()
      {
         long lastSendTimestamp = Long.MIN_VALUE;

         while (active)
         {
            // Localize variable so it doesn't change in this thread
            long newTimestampLocal = newTimestamp;

            if (lastSendTimestamp != newTimestampLocal)
            {

               // Localize variables
               WebsocketDataServerFrameHandler[] localChannels = channels;

               for (int i = 0; i < localChannels.length; i++)
               {
                  localChannels[i].publishTimestamp(newTimestampLocal);
               }
               
               lastSendTimestamp = newTimestampLocal;
            }

            LockSupport.parkNanos(TIMESTAMP_PUBLISHING_SLEEP_NS);
         }

      }

   }

}
