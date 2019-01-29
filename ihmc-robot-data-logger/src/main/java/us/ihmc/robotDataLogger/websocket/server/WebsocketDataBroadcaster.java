package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

public class WebsocketDataBroadcaster
{
   private final Object channelLock = new Object();

   private final ArrayList<WebsocketLogFrameHandler> channels = new ArrayList<WebsocketLogFrameHandler>();

   public WebsocketDataBroadcaster()
   {
   }

   public void addClient(WebsocketLogFrameHandler websocketLogFrameHandler)
   {

      System.out.println("Adding new channel {} to list of channels " + websocketLogFrameHandler.remoteAddress());

      synchronized (channelLock)
      {
         channels.add(websocketLogFrameHandler);
      }

      System.out.println(channels);
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

}
