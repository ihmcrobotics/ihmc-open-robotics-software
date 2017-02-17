package us.ihmc.communication.net;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.KryoSerialization;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Serialization;

import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class KryoObjectClient extends KryoObjectCommunicator
{
   private int timeOut = 1000;
   private int reconnectDelay = 1000;
   private final InetAddress host;
   private final int tcpPort;

   private final int writeBufferSize; 
   
   private boolean reconnectAutomatically = false;
   private boolean isClosed = true;

   private final Client client;

   public static InetAddress getByName(String host)
   {
      try
      {
         return InetAddress.getByName(host);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public KryoObjectClient(String host, int tcpPort, NetClassList netClassList)
   {
      this(getByName(host), tcpPort, netClassList, 2097152, 2097152);
   }

   public KryoObjectClient(InetAddress host, int tcpPort, NetClassList netClassList, int writeBufferSize, int receiveBufferSize)
   {
      this(host, tcpPort, netClassList, writeBufferSize, receiveBufferSize, new KryoSerialization());
      netClassList.registerWithKryo(client.getKryo());
   }

   public KryoObjectClient(InetAddress host, int tcpPort, NetClassList netClassList, int writeBufferSize, int receiveBufferSize, Serialization serialization)
   {
      super();
      this.client = new Client(writeBufferSize, receiveBufferSize, serialization);
      this.host = host;
      this.tcpPort = tcpPort;
      this.writeBufferSize = writeBufferSize;

      registerClassList(netClassList);
      createConnectionListener(client);
      createReconnectListener();

   }

   public void setTimeOut(int timeOutMillis)
   {
      this.timeOut = timeOutMillis;
   }

   public void setDelayForReconnect(int delayMillis)
   {
      this.reconnectDelay = delayMillis;
   }

   private void createReconnectListener()
   {
      client.addListener(new Listener()
      {
         @Override
         public void disconnected(Connection connection)
         {
            if (reconnectAutomatically && !isClosed)
            {
               openConnectionOnAThread();
            }
         }
      });
   }

   public void setReconnectAutomatically(boolean reconnectAutomatically)
   {
      this.reconnectAutomatically = reconnectAutomatically;
   }

   @Override
   protected int sendUDP(Object object)
   {
      return client.sendUDP(object);
   }

   @Override
   protected int sendTCP(Object object)
   {
      // Do not send if the write buffer > 90% full
      if (((double) client.getTcpWriteBufferSize()) / ((double) writeBufferSize) > 0.9)
      {
         return 0;
      }
      return client.sendTCP(object);
   }

   @Override
   public boolean isConnected()
   {
      return client.isConnected();
   }

   @Override
   public void closeConnection()
   {
      isClosed = true;
      client.stop();
   }

   private void doConnect() throws IOException
   {
      client.connect(timeOut, host, tcpPort);
      PrintTools.info(this, "Success! Connected KryoClient to SCS at ip " + host + " on port " + tcpPort);
   }

   private void openConnectionOnAThread()
   {
      Thread connectionThread = new Thread(new Runnable()
      {

         public void run()
         {
            while (!client.isConnected() && !isClosed)
            {
               try
               {
                  doConnect();
               }
               catch (IOException e)
               {
                  //                  System.err.println("Failed to connect KryoClient to SCS at ip: " + host + " at port: " + tcpPort + ". retrying...");
                  ThreadTools.sleep(reconnectDelay);
               }
            }
         }
      });
      connectionThread.start();
   }

   @Override
   protected void openConnection() throws IOException
   {
      isClosed = false;
      client.start();
      if (reconnectAutomatically)
      {
         openConnectionOnAThread();
      }
      else
      {
         doConnect();
      }
   }
}
