package us.ihmc.communication.remote;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;

import us.ihmc.tools.thread.ThreadTools;

/**
 * User: Matt
 * Date: 12/7/12
 */
public class DataObjectServer extends DataObjectTransponder
{
   private ServerSocket serverSocket = null;
   private int port;
   private String host;

   public DataObjectServer()
   {
      this(null, 0);
   }

   public DataObjectServer(int port)
   {
      this(null, port);
   }

   public DataObjectServer(String host, int port)
   {
      init(host, port);
   }



   static DataObjectTransponder initByInterface(String interfaceName, int iPort)
   {
      try
      {
         if (DEBUG)
         {
            Enumeration<NetworkInterface> inets = NetworkInterface.getNetworkInterfaces();
            System.out.println("Network interfaces");

            while (inets.hasMoreElements())
            {
               System.out.println(inets.nextElement());
               System.out.println("--------------------------------");
            }
         }

         NetworkInterface myNetworkInterface = NetworkInterface.getByName(interfaceName);
         printIfDebug("interfaceName = " + myNetworkInterface);


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

                  printIfDebug("Found address for " + interfaceName + ": " + currentInetAddress.getHostAddress());

               }
            }
         }

         return new DataObjectServer(host, iPort);
      }
      catch (SocketException ex)
      {
         ex.printStackTrace();
      }

      return null;
   }

   /**
    * Tries to start a server on the specified port.
    *
    * @param host - host name (IP) to use
    * @param port - port to use for opened socket
    */
   private void init(String host, int port)
   {
      if (host != null)
      {
         this.host = host;
      }
      else
      {
         try
         {
            this.host = InetAddress.getLocalHost().getHostAddress();
         }
         catch (UnknownHostException xcp)
         {
            xcp.printStackTrace();
            this.host = null;
         }
      }

      try
      {
         serverSocket = new ServerSocket(port);
      }
      catch (IOException xcp)
      {
         handleFailureToCreateServerSocket(port, xcp);
         return;
      }

      // Get port being used for server socket
      this.port = serverSocket.getLocalPort();

      // Create a server thread and start it listening
      Runnable serverConnectionDaemon = new ServerConnectionDaemon();
      ThreadTools.startAsDaemon(serverConnectionDaemon, "Data Object Server Conn Daemon");

      printIfDebug(".init: comm server started listening on:\n" + "\thost = " + this.host + "\n\tport = " + this.port);

   }

   private void handleFailureToCreateServerSocket(int port, IOException xcp)
   {
      printIfDebug(".init: Error Creating Server Socket on port: " + port);


      xcp.printStackTrace();
      serverSocket = null;
      this.port = -1;
   }

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

   public String getHost()
   {
      return host;
   }

   public int getPort()
   {
      return port;
   }

   // Server Thread - Listens for new connections and add them to the list of connections to handle
   private class ServerConnectionDaemon implements Runnable
   {
      public void run()
      {
         while ((serverSocket != null) &&!serverSocket.isClosed())
         {
            if (!isConnected())
            {
               try
               {
                  setupNextClient();
               }
               catch (SocketException xcp)
               {
                  if (!xcp.getMessage().equals("socket closed"))
                  {
                     xcp.printStackTrace();
                  }
               }
               catch (IOException xcp)
               {
                  xcp.printStackTrace();
               }
            }
         }
         printIfDebug(".ServerThread: ServerAccept Thread has Stopped.");
      }

      private void setupNextClient() throws IOException
      {
         printIfDebug(".ServerThread_" + port + ": Waiting for next client");
         Socket clientSocket = serverSocket.accept();
         connectToSocket(clientSocket);

         printIfDebug(".ServerThread_" + port + ": Got a connection handling on Port: " + clientSocket.getPort());
         printIfDebug("added " + clientSocket.getPort());
      }
   }
}
