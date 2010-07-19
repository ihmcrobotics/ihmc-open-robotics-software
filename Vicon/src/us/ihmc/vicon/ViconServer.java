package us.ihmc.vicon;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;

public class ViconServer extends ViconJavaInterface
{
   protected ServerSocket serverSocket = null;
   protected int port;
   protected String host;
   private boolean DEBUG = false;

   public ViconServer()
   {
      // connect to vicon
      ViconConnect("10.4.1.100");

      // get list of available bodies
      ViconGetFrame();
      int numBodies = ViconGetNumBodies();
      System.out.println("available models:");

      for (int i = 0; i < numBodies; i++)
      {
         String name = ViconGetBodyName(i);
         System.out.println("\t" + name);
      }

      // start tcp server
      init(7777);
      System.out.println("Vicon server started on " + getHost() + ": " + getPort());
   }

   public void init()
   {
      // NO port specified
      init(null, 0);
   }

   public void init(int port)
   {
      // NO port specified
      init(null, port);
   }

   public void initByInterface(String interfaceName, int iPort)
   {
      try
      {
         NetworkInterface myNetworkInterface = NetworkInterface.getByName(interfaceName);
         if (DEBUG)
            System.out.println("interfaceName = " + myNetworkInterface);

         // Find IP address category 4 for this network interface
         boolean foundHost = false;
         String host = null;
         if (myNetworkInterface != null)
         {
            for (Enumeration<InetAddress> myAddresses = myNetworkInterface.getInetAddresses(); myAddresses.hasMoreElements() &&!foundHost; )
            {
               InetAddress currentInetAddress = myAddresses.nextElement();

               if (currentInetAddress instanceof Inet4Address)
               {
                  host = ((Inet4Address) currentInetAddress).getHostAddress();
                  foundHost = true;

                  if (DEBUG)
                     System.out.println("Found address for " + interfaceName + ": " + currentInetAddress.getHostAddress());
               }
            }
         }

         init(host, iPort);
      }
      catch (SocketException ex)
      {
         ex.printStackTrace();
      }
   }

   /**
    * Tries to start a socketserver on the specified port.
    *
    * @param host - host name (IP) to use
    * @param port - port to use for opened socket
    */
   public void init(String host, int iPort)
   {
      if (host != null)
         this.host = host;
      else
         try
         {
            this.host = InetAddress.getLocalHost().getHostAddress();
         }
         catch (UnknownHostException xcp)
         {
            xcp.printStackTrace();
            this.host = null;
         }

      // TODO: Inet4Address hostInet4Address = new Inet4Address(null, textToNumericFormat(_host));

      // Try to create server socket
      try
      {
         serverSocket = new ServerSocket(iPort);    // TODO: use hostInet4Address
      }
      catch (IOException xcp)
      {
         if (DEBUG)
            System.out.println(".init: Error Creating Server Socket on port: " + iPort);

         xcp.printStackTrace();
         serverSocket = null;
         port = -1;

         return;
      }

      // Get port being used for server socket
      port = serverSocket.getLocalPort();

      // Create a server thread and start it listening
      Thread svrThrd = new Thread(new ServerThread());
      svrThrd.setDaemon(true);
      svrThrd.start();

      if (DEBUG)
         System.out.println(".init: comm server started listening on:\n" + "\thost = " + this.host + "\n\tport = " + port);
   }

   /**
    * Tries to close server socket
    */
   public void close()
   {
      try
      {
         serverSocket.close();
      }
      catch (Exception xcp)
      {
         // Do nothing
      }

      serverSocket = null;
      port = -1;
      host = null;
   }

   /**
    * Returns the hostname of this server.
    */
   public String getHost()
   {
      return host;
   }

   /**
    * Returns the port number of this server.
    */
   public int getPort()
   {
      return port;
   }

   // ------------- Server Thread - Listens for new connections and add them to the list of connections to handle
   public class ServerThread implements Runnable
   {
      public void run()
      {
         while ((serverSocket != null) &&!serverSocket.isClosed())
         {
            try
            {
               // Wait for client requests
               if (DEBUG)
                  System.out.println(".ServerThread_" + port + ": Waiting for next client");

               Socket clientSocket = serverSocket.accept();

               // Create a handler for new clients
               if (DEBUG)
                  System.out.println(".ServerThread_" + port + ": Got a connection handling on Port: " + clientSocket.getPort());

               ClientHandler ch = new ClientHandler(clientSocket);

               ch.start();

            }
            catch (SocketException xcp)
            {
               if (!xcp.getMessage().equals("socket closed"))
                  xcp.printStackTrace();
            }
            catch (IOException xcp)
            {
               xcp.printStackTrace();
            }
         }

         if (DEBUG)
            System.out.println(".ServerThread: ServerAccept Thread has Stopped.");
      }
   }


   // -------- Client Thread
   public class ClientHandler implements Runnable
   {
      protected Socket clientSocket = null;
      protected DataInputStream dataInputStream = null;
      protected DataOutputStream dataOutputStream = null;

      protected boolean _disconnected = false;
      protected int clientPort = -1;
      protected String clientHost = null;

      /**
       * ClientHandler sets the socket
       *
       * @param the socket for the client handler to read from
       */
      public ClientHandler(Socket clientSocket)
      {
         this.clientSocket = clientSocket;
         clientPort = clientSocket.getPort();
      }

      /**
       * starts the client handler
       */
      public void start()
      {
         if (clientSocket == null)
         {
            _disconnected = true;
            if (DEBUG)
               System.out.println("client socket is null");

            return;
         }

         Thread thrd = new Thread(this);
         thrd.setDaemon(true);
         thrd.start();
      }

      /**
       * stop - sets client handler to disconnected which will terminate the "run" while loop
       */
      public void stop()
      {
         _disconnected = true;
      }

      /**
       * run - while connected, continues to read the object input stream and adds all objects to the newObjects vector
       */
      public void run()
      {
         try
         {
            dataInputStream = new DataInputStream(new BufferedInputStream(clientSocket.getInputStream()));
            dataOutputStream = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));
         }
         catch (IOException xcp)
         {
            dataInputStream = null;
            _disconnected = true;
            xcp.printStackTrace();

            return;
         }

         while (!_disconnected)
         {
            try
            {
//             long start = System.currentTimeMillis();
               @SuppressWarnings("unused")
               byte unnecessaryValue = dataInputStream.readByte();

//             long read = 0, invoke = 0, reply = 0;
//             read = System.currentTimeMillis() - start;

               try
               {
                  Pose pose = getPose("biped:biped");

//                invoke = System.currentTimeMillis() - (start + read);

                  if (DEBUG)
                     System.out.println(".ClientHandler: returning " + pose);


                  dataOutputStream.writeFloat(pose.xPosition);
                  dataOutputStream.writeFloat(pose.yPosition);
                  dataOutputStream.writeFloat(pose.zPosition);
                  dataOutputStream.writeFloat(pose.xAxisRotation);
                  dataOutputStream.writeFloat(pose.yAxisRotation);
                  dataOutputStream.writeFloat(pose.zAxisRotation);

                  dataOutputStream.flush();

//                reply = System.currentTimeMillis() - (start + read + invoke);

               }
               catch (Exception ex)
               {
                  ex.printStackTrace();
               }

//             System.out.println("read=" + read + "    invoke=" + invoke + "   reply=" + reply + "    objectName=" +
//                                objectName + "    pose=" + pose);
            }
            catch (IOException xcp)
            {
               _disconnected = true;
            }
         }

         try
         {
            clientSocket.close();
         }
         catch (IOException xcp)
         {
         }

         dataInputStream = null;
         clientSocket = null;
      }

      /**
       * isConnected
       *
       * @return        a boolean true = connected, false = NOT connected
       */
      public boolean isConnected()
      {
         return !_disconnected;
      }

      /**
       * getHost
       *
       * @return        host name
       */
      public String getHost()
      {
         return clientHost;
      }

      /**
       * getPort
       *
       * @return        port number
       */
      public int getPort()
      {
         return clientPort;
      }
   }


   public static void main(String[] args)
   {
      @SuppressWarnings("unused")
      ViconServer viconserver = new ViconServer();

      // wait around until terminated
      synchronized (Thread.currentThread())
      {
         try
         {
            Thread.currentThread().wait();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }

   }
}
