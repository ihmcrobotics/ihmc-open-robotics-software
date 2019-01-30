package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelFutureListener;

public class WebsocketDataBroadcaster implements ChannelFutureListener
{
   private final Object channelLock = new Object();

   private final ArrayList<WebsocketDataServerFrameHandler> channels = new ArrayList<WebsocketDataServerFrameHandler>();

   public WebsocketDataBroadcaster()
   {
   }

   public void addClient(WebsocketDataServerFrameHandler websocketLogFrameHandler)
   {

      synchronized (channelLock)
      {
         channels.add(websocketLogFrameHandler);
         websocketLogFrameHandler.addCloseFutureListener(this);
      }
   }

   public void write(ByteBuffer frame) throws IOException
   {
      synchronized (channelLock)
      {
         for (int i = 0; i < channels.size(); i++)
         {
            channels.get(i).write(frame);
         }
      }

   }

   @Override
   public void operationComplete(ChannelFuture future) throws Exception
   {
      synchronized (channelLock)
      {
         for (int i = 0; i < channels.size(); i++)
         {
            if(channels.get(i).channel() == future.channel())
            {
               System.out.println("Client disconnect " + future.channel());
               channels.remove(i).release();
               return;
            }
         }
      }
   }

}
