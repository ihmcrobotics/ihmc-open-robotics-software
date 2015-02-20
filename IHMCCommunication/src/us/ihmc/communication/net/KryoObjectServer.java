package us.ihmc.communication.net;

import java.io.IOException;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.KryoSerialization;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Serialization;
import com.esotericsoftware.kryonet.Server;

public class KryoObjectServer extends KryoObjectCommunicator
{
   private final Server server;

   private final int tcpPort;

   private int maximumNumberOfConnections = Integer.MAX_VALUE;

   public KryoObjectServer(int tcpPort, NetClassList netClassList)
   {
      this(tcpPort, netClassList, 2097152, 2097152);
   }

   public KryoObjectServer(int tcpPort, NetClassList netClassList, int writeBufferSize, int receiveBufferSize)
   {
      this(tcpPort, netClassList, writeBufferSize, receiveBufferSize, new KryoSerialization());
      netClassList.registerWithKryo(server.getKryo());
   }

   public KryoObjectServer(int tcpPort, NetClassList netClassList, int writeBufferSize, int receiveBufferSize, Serialization serialization)
   {
      super();
      this.server = new Server(writeBufferSize, receiveBufferSize, serialization);
      this.tcpPort = tcpPort;

      registerClassList(netClassList);
      createConnectionListener(server);
      createConnectionLimiter();
   }

   private void createConnectionLimiter()
   {
      server.addListener(new Listener()
      {
         @Override
         public void connected(Connection connection)
         {
            if (getNumberOfConnections() > maximumNumberOfConnections)
            {
               connection.close();
            }
         }
      });
   }

   public void setMaximumNumberOfConnections(int maximumNumberOfConnections)
   {
      this.maximumNumberOfConnections = maximumNumberOfConnections;
   }

   @Override
   protected int sendUDP(Object object)
   {
      int bytesSend = 0;
      Connection[] connections = server.getConnections();
      for (int i = 0, n = connections.length; i < n; i++)
      {
         Connection connection = connections[i];
         bytesSend = connection.sendUDP(object);
      }

      return bytesSend;
   }

   @Override
   protected int sendTCP(Object object)
   {
      int bytesSend = 0;
      Connection[] connections = server.getConnections();
      for (int i = 0, n = connections.length; i < n; i++)
      {
         Connection connection = connections[i];
         bytesSend = connection.sendTCP(object);
      }

      return bytesSend;
   }

   @Override
   public boolean isConnected()
   {
      for (Connection connection : server.getConnections())
      {
         if (connection.isConnected())
         {
            return true;
         }
      }
      return false;
   }

   public int getNumberOfConnections()
   {
      int numberOfConnections = 0;
      for (Connection connection : server.getConnections())
      {
         if (connection.isConnected())
         {
            numberOfConnections++;
         }
      }
      return numberOfConnections;
   }

   @Override
   public void closeConnection()
   {
      server.stop();
   }

   @Override
   protected void openConnection() throws IOException
   {
      server.start();
      server.bind(tcpPort);
   }
}
