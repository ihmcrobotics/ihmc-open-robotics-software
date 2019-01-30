package us.ihmc.robotDataLogger.websocket.client;

import us.ihmc.robotDataLogger.Announcement;

public interface HTTPDataServerConnectionListener
{
   public void connected(HTTPDataServerConnection connection, Announcement announcement);
   
   public void disconnected(HTTPDataServerConnection connection);
   
   public void connectionRefused();
}
