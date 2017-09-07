package us.ihmc.communication.remote;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

import us.ihmc.communication.interfaces.Connectable;

public class RemoteConnection implements Connectable
{
   private static final int PORT = 7733;
   
   private boolean isConnected = false;
   protected Socket socket = null;
   protected OutputStream os = null;
   protected InputStream is = null;
   protected ObjectOutputStream oos = null;
   protected ObjectInputStream ois = null;
   private String myIP;
   private boolean DEBUG = false;

   @Override
   public boolean isConnected()
   {
      return isConnected;
   }

   public void connect() throws Exception
   {
      connect("localhost", PORT);
   }

   public void connect(String host) throws Exception
   {
      connect(host, PORT);
   }

   public void connectAsync(String host) throws Exception
   {
      ConnectionThread connectionThread = new ConnectionThread(host, PORT);
      connectionThread.setDaemon(true);
      connectionThread.start();
   }


   /**
    * Tries to connect to the specified host on the specified port and open an object output stream to it.
    *
    * @param host
    * @param port
    */
   public void connect(String host, int port) throws Exception
   {
      if (DEBUG)
         System.out.println(".CommClient: initializing comm client to: \n" + "\thost = " + host + "\n\tport = " + port);

      try
      {
         InetAddress i = java.net.InetAddress.getLocalHost();
         myIP = i.getHostAddress();
      }
      catch (UnknownHostException ex)
      {
         ex.printStackTrace();
      }

      // If this is local, use 127.0.0.1 to avoid problem with disconnecting; cord causing comms failure with local agents
      if (host.equals(myIP))
      {
         host = "127.0.0.1";
         if (DEBUG)
            System.out.println(".CommClient: This is my IP...use local host");
      }

      // Create socket
      try
      {
         socket = new Socket(host, port);
      }
      catch (UnknownHostException xcp)
      {
         socket = null;

         throw xcp;
      }
      catch (IOException xcp)
      {
         socket = null;

         throw xcp;
      }

      // Get outputstream from socket
      try
      {
         os = new BufferedOutputStream(socket.getOutputStream());
         is = new BufferedInputStream(socket.getInputStream());
         oos = new ObjectOutputStream(socket.getOutputStream());
         ois = new ObjectInputStream(socket.getInputStream());
      }
      catch (IOException xcp)
      {
         // xcp.printStackTrace();

         try
         {
            socket.close();
         }
         catch (IOException xcp2)
         {
            // ignore
         }

         oos = null;
         socket = null;

         throw xcp;
      }

      isConnected = true;
   }

   /**
    * Closes any open connections.
    */
   public void close()
   {
      if (socket != null)
      {
         try
         {
            socket.close();
         }
         catch (IOException xcp2)
         {
            // ignore
         }

         oos = null;
         socket = null;
      }

      isConnected = false;
   }

   public synchronized void SendString(String string) throws Exception
   {
      if (oos == null)
      {
         if (DEBUG)
            System.out.println("SendString: output stream is null");

         return;
      }

      oos.writeUTF(string);
      oos.flush();
   }

   /**
    * Sends the specified Serializable object across the ObjectOutputStream.
    *
    * @param remoteRequest
    */
   public synchronized void SendObject(RemoteRequest remoteRequest) throws Exception
   {
      if (oos == null)
      {
         if (DEBUG)
            System.out.println("SendObject: output stream is null for " + remoteRequest);

         return;
      }

      oos.writeObject(remoteRequest);
      oos.flush();
      oos.reset();
   }

   public synchronized Object SendRequest(RemoteRequest remoteRequest) throws Exception
   {
      if (oos == null)
      {
         if (DEBUG)
            System.out.println("SendRequest: output stream is null for " + remoteRequest);

         return null;
      }

      long startTime = System.currentTimeMillis();
      oos.writeObject(remoteRequest);
      oos.flush();
      oos.reset();
      long writeTime = System.currentTimeMillis();

//    ObjectInputStream ois = new ObjectInputStream(socket.getInputStream());
      Object result = ois.readObject();
      long endTime = System.currentTimeMillis();
      if(DEBUG)
      {
         System.out.println("sendRequest took: ");
         System.out.println("\twrite = " + (writeTime-startTime) + " ms");
         System.out.println("\tread = " + (endTime-writeTime) + " ms");
      }

      return result;
   }

   class ConnectionThread extends Thread
   {
      private String host;
      private int port;

      public ConnectionThread(String host, int port)
      {
         this.host = host;
         this.port = port;
      }

      @Override
      public void run()
      {
         try
         {
            connect(host, port);
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         }
      }
   }
}
