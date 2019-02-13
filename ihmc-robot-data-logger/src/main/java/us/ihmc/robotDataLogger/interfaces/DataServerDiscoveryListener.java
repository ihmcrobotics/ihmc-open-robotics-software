package us.ihmc.robotDataLogger.interfaces;

import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

public interface DataServerDiscoveryListener
{
   public void connected(HTTPDataServerConnection connection);
   public void disconnected(HTTPDataServerConnection connection);

}
