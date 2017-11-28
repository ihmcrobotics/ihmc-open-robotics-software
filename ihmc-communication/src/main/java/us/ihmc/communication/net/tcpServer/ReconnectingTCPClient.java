package us.ihmc.communication.net.tcpServer;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import us.ihmc.commons.thread.ThreadTools;

public class ReconnectingTCPClient extends ReconnectingTCPConnection
{
   private final String host;
   private final int port;

   private Socket clientSocket;

   public ReconnectingTCPClient(String host, int port)
   {
      super(2097152);
      this.host = host;
      this.port = port;
   }

   @Override
   public void disconnect()
   {
      synchronized (connectionStatusSync)
      {
         try
         {
            inputStream.close();
            outputStream.close();
            clientSocket.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         setStatus(Status.DISCONNECTED);
         notifyDisconnectedListeners();
      }
   }

   @Override
   public void connect()
   {
      if(getStatus() == Status.CLOSED)
      {
         throw new RuntimeException("Connection is closed");
      }
      
      while (getStatus() == Status.DISCONNECTED)
      {
         try
         {
            clientSocket = new Socket(host, port);
            synchronized (connectionStatusSync)
            {
               clientSocket.setTcpNoDelay(true);
               inputStream = new DataInputStream(clientSocket.getInputStream());
               outputStream = clientSocket.getOutputStream();
               reset();
               setStatus(Status.CONNECTED);
               notifyConnectedListeners();               
            }
         }
         catch (UnknownHostException e)
         {
            throw new RuntimeException(e);
         }
         catch (IOException e)
         {
            System.out.println("Reconnecting to " + host + ":" + port + "...");
            ThreadTools.sleep(1000);
         }
      }
      
   }

   @Override
   public void close()
   {
      synchronized (connectionStatusSync)
      {
         disconnect();
         setStatus(Status.CLOSED);
      }
   }

}
