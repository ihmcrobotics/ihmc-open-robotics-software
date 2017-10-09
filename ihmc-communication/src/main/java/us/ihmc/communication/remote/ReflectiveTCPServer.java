package us.ihmc.communication.remote;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;
import java.util.Vector;

public class ReflectiveTCPServer
{
   private Object reflectiveObject;
   protected ServerSocket serverSocket = null;
   protected int port;
   protected String host;
   private boolean DEBUG = false;

   public ReflectiveTCPServer(Object reflectiveObject)
   {
      this.reflectiveObject = reflectiveObject;
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
//       Enumeration inets = NetworkInterface.getNetworkInterfaces();
//       while(inets.hasMoreElements())
//       {
//           System.out.println(inets.nextElement());
//           System.out.println("--------------------------------");
//       }

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
   public void init(String host, int port)
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
         serverSocket = new ServerSocket(port);    // TODO: use hostInet4Address
      }
      catch (IOException xcp)
      {
         if (DEBUG)
            System.out.println(".init: Error Creating Server Socket on port: " + port);

         xcp.printStackTrace();
         serverSocket = null;
         this.port = -1;

         return;
      }

      // Get port being used for server socket
      this.port = serverSocket.getLocalPort();

      // Create a server thread and start it listening
      Thread svrThrd = new Thread(new ServerThread());
      svrThrd.setDaemon(true);
      svrThrd.start();

      if (DEBUG)
         System.out.println(".init: comm server started listening on:\n" + "\thost = " + this.host + "\n\tport = " + this.port);
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
      @Override
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
      protected Socket _clientSocket = null;
      protected ObjectInputStream _ois = null;
      protected ObjectOutputStream _oos = null;

      protected boolean _disconnected = false;
      protected int _clientPort = -1;
      protected String _clientHost = null;

      /**
       * ClientHandler sets the socket
       *
       * @param clientSocket      the socket for the client handler to read from
       */
      public ClientHandler(Socket clientSocket)
      {
         _clientSocket = clientSocket;
         _clientPort = clientSocket.getPort();
      }

      /**
       * starts the client handler
       */
      public void start()
      {
         if (_clientSocket == null)
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
      @Override
      public void run()
      {
         try
         {
            _ois = new ObjectInputStream(_clientSocket.getInputStream());
            _oos = new ObjectOutputStream(_clientSocket.getOutputStream());

         }
         catch (IOException xcp)
         {
            _ois = null;
            _disconnected = true;
            xcp.printStackTrace();

            return;
         }

         while (!_disconnected)
         {
            try
            {
               long start = System.currentTimeMillis();
               Object obj = _ois.readObject();
               long read = 0, invoke = 0, reply = 0;
               read = System.currentTimeMillis() - start;
               if (DEBUG)
                  System.out.println(".ClientHandler: received object " + obj);

               // Update message listeners
               if (obj instanceof RemoteRequest)
               {
                  RemoteRequest remoteRequest = (RemoteRequest) obj;
                  if (DEBUG)
                     System.out.println(".ClientHandler: received remote request " + remoteRequest);

                  try
                  {
                     Object result = invokeMethod(remoteRequest.methodName, remoteRequest.parameters);
                     invoke = System.currentTimeMillis() - (start + read);

                     if (result instanceof Serializable)
                     {
                        if (DEBUG)
                           System.out.println(".ClientHandler: returning " + result);

                        _oos.writeObject(result);

//                      _oos.flush();
                        reply = System.currentTimeMillis() - (start + read + invoke);
                     }
                     else
                     {
                        if (result != null)
                           System.out.println(remoteRequest.methodName + " cannot return non serializable: " + result.getClass());
                     }
                  }
                  catch (Exception ex)
                  {
                     ex.printStackTrace();
                  }
               }
               else
               {
                  if (DEBUG)
                     System.out.println("received obj which is not a RemoteRequest: " + obj.getClass());
               }

               if(DEBUG) System.out.println("read=" + read + "    invoke=" + invoke + "   reply=" + reply + "    obj=" + obj);
            }
            catch (IOException xcp)
            {
               _disconnected = true;
            }
            catch (ClassNotFoundException xcp)
            {
               _disconnected = true;
            }
         }

         try
         {
            _clientSocket.close();
         }
         catch (IOException xcp)
         {
         }

         _ois = null;
         _clientSocket = null;
      }

      /**
       * isConnected
       *
       * @return      a boolean true = connected, false = NOT connected
       */
      public boolean isConnected()
      {
         return !_disconnected;
      }

      /**
       * getHost
       *
       * @return      host name
       */
      public String getHost()
      {
         return _clientHost;
      }

      /**
       * getPort
       *
       * @return      port number
       */
      public int getPort()
      {
         return _clientPort;
      }
   }


   public Object invokeMethod(String methodName, Vector<?> args) throws Exception
   {
      try
      {
         if (args != null)
         {
            Object argObjects[] = new Object[args.size()];
            for (int i = 0; i < args.size(); i++)
            {
               argObjects[i] = args.elementAt(i);
            }

            // We have to check this the "hard way" because Class.getMethod(name, args[]) does not work if the args are subclasses of the parameter classes

            // Find the method matching the method name requested
            Method methods[] = reflectiveObject.getClass().getMethods();
            boolean foundMethod = false;

            for (int i = 0; i < methods.length; i++)
            {
               if (methods[i].getName().equals(methodName))
               {
                  // Check to see if the parameter classes match the arguments
                  Class<?> parameters[] = methods[i].getParameterTypes();
                  boolean classesMatch = false;
                  if (parameters.length == argObjects.length)
                  {
                     classesMatch = true;

                     for (int j = 0; j < parameters.length; j++)
                     {
                        // bug fix: check for null case
                        // if the arg is null, or if it is an instance of the parameter's class,
                        // we are OK; otherwise break
                        if ((argObjects[j] != null) &&!parameters[j].isInstance(argObjects[j]))
                        {
                           classesMatch = false;

                           break;
                        }
                     }
                  }

                  // If we found the right method, execute it
                  if (classesMatch)
                  {
                     foundMethod = true;
                     Object result = methods[i].invoke(reflectiveObject, argObjects);

                     return result;
                  }
               }
            }

            // If we didn't find the method, throw an exception
            if (!foundMethod)
               throw new NoSuchMethodException(methodName);
         }
         else    // if args == null, invoke method with no args
         {
            Object result = reflectiveObject.getClass().getMethod(methodName, (Class[]) null).invoke(reflectiveObject, (Object[]) null);

            return result;
         }
      }
      catch (InvocationTargetException inex)
      {
         throw(Exception) inex.getTargetException();
      }

      return null;
   }

   public static void main(String[] args)
   {
   }
}
