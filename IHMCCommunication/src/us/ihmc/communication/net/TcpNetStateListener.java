package us.ihmc.communication.net;

import com.esotericsoftware.kryonet.Connection;

public interface TcpNetStateListener
{
   void connected(Connection connection);
   void disconnected(Connection connection);
}
