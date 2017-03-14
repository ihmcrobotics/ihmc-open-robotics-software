package us.ihmc.communication.net;

import java.io.IOException;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.ByteBufferOutputStream;
import com.esotericsoftware.kryo.io.Output;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.KryoSerialization;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Serialization;
import com.esotericsoftware.kryonet.Server;

import us.ihmc.commons.PrintTools;

public class KryoObjectServer extends KryoObjectCommunicator
{
   private final int writeBufferSize;
   private final Server server;
   private final int tcpPort;

   private int maximumObjectSize = 0;
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
      this.server = new Server(writeBufferSize, receiveBufferSize, serialization);
      this.writeBufferSize = writeBufferSize;
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

   public void setMaximumObjectSize(int maximumObjectSize)
   {
      this.maximumObjectSize = maximumObjectSize;
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

   /** This method produces a lot of garbage. Use it wisely! */
   public static int calculateObjectSize(Object object, int bufferSize)
   {
      Kryo kryo = new Kryo();
      ByteBufferOutputStream bbos = new ByteBufferOutputStream(bufferSize);
      Output output = new Output(bbos, bufferSize);
      kryo.writeClassAndObject(output, object);
      output.flush();
      output.close();
      return bbos.getByteBuffer().position();
   }

   @Override
   protected int sendTCP(Object object)
   {
      int bytesSend = 0;

      // Do not send if the object is above the limit
      if (maximumObjectSize > 0)
      {
         int size = calculateObjectSize(object, writeBufferSize);
         if (size > maximumObjectSize)
         {
            PrintTools.error(this, "Dropping the object of " + object.getClass() + ", because it is too big " + size + " > " + maximumObjectSize);
            return 0;
         }
      }

      Connection[] connections = server.getConnections();
      for (int i = 0, n = connections.length; i < n; i++)
      {
         Connection connection = connections[i];

         // Do not send if the write buffer > 90% full
         if (((double) connection.getTcpWriteBufferSize()) / ((double) writeBufferSize) > 0.9)
         {
            PrintTools.error(this, "Dropping the object of " + object.getClass() + ", because the write buffer is > 90% full!");
            continue;
         }

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
