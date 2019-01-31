package us.ihmc.robotDataLogger.websocket.client.discovery;

import us.ihmc.robotDataLogger.Announcement;

public interface DataServerDiscoveryListener
{
   public void connected(HTTPDataServerConnection connection);
   public void disconnected(HTTPDataServerConnection connection);

}
