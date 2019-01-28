package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;

import io.netty.buffer.ByteBuf;
import io.netty.channel.Channel;
import io.netty.channel.group.ChannelGroup;
import io.netty.channel.group.ChannelMatcher;
import io.netty.channel.group.DefaultChannelGroup;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.util.concurrent.GlobalEventExecutor;

public class WebsocketDataBroadcaster
{
   private final ChannelGroup clients = new DefaultChannelGroup(GlobalEventExecutor.INSTANCE);
   private static final ChannelMatcher WRITABLE_CHANNEL_MATCHER = new WritableChannelMatcher();

   
   public WebsocketDataBroadcaster()
   {
   }
   
         
         
         
   private static class WritableChannelMatcher implements ChannelMatcher
   {

      @Override
      public boolean matches(Channel channel)
      {
         if (channel.isWritable())
         {
            return true;
         }
         else
         {
            return false;
         }
      }

   }

   public void addClient(Channel channel)
   {

      System.out.println("Adding new channel {} to list of channels " + channel.remoteAddress());
      clients.add(channel);
      System.out.println(clients);
   }

   public void write(ByteBuf sendBuffer) throws IOException
   {

      
      BinaryWebSocketFrame frame = new BinaryWebSocketFrame(sendBuffer);
      clients.writeAndFlush(frame, WRITABLE_CHANNEL_MATCHER, true);
      
      
      
   }

}
