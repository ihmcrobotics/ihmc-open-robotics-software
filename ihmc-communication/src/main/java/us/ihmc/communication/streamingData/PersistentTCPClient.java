package us.ihmc.communication.streamingData;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commons.thread.ThreadTools;

public class PersistentTCPClient
{
   private final String serverIPAddress;
   private final int serverPort;
   private final EstablishedAConnectionListener establishedAConnectionListener;

   private Socket socket;
   private InputStream inputStream;
   private ObjectInputStream objectInputStream;

   private AtomicBoolean connected = new AtomicBoolean(false);
   
   private boolean keepOnTrying = false;
   
   private static final boolean DEBUG = false;
   
   public PersistentTCPClient(String serverIPAddress, int serverPort, EstablishedAConnectionListener establishedAConnectionListener)
   {
      this.serverIPAddress = serverIPAddress;
      this.serverPort = serverPort;
      
      this.establishedAConnectionListener = establishedAConnectionListener;
   }

   public void connectToServer(boolean persistentlyStayConnected)
   {
      if (persistentlyStayConnected)
      {
         keepOnTrying = true;

         StreamingDataTCPClientPersistentConnector streamingDataTCPClientPersistentConnector = new StreamingDataTCPClientPersistentConnector();
         Thread thread = new Thread(streamingDataTCPClientPersistentConnector);
         thread.setDaemon(true);
         thread.start();
      }

      else
      {
         attemptToCreateSocketAndStreams();
         if (!connected.get())
            return;

         establishedAConnectionListener.establishedAConnection(objectInputStream, null);
      }
   }
   
   
   private class StreamingDataTCPClientPersistentConnector implements Runnable
   {
      public void run()
      {
         while (keepOnTrying)
         {
            if (!(isConnected()))
            {
               printIfDebug("!! Trying To Connect !!");

               attemptToCreateSocketAndStreams();
               if (socket != null)
               {
                  establishedAConnectionListener.establishedAConnection(objectInputStream, null);
               }
            }
            
            while(isConnected())
            { 
               ThreadTools.sleep(100L);
            }

            printIfDebug("!! Closing Streams and Sockets !!");
            
            closeStreamsAndSockets();
            // Sleep and try seeing if the server is available in a couple seconds.
            ThreadTools.sleep(100L);
         }
      }
   }
   
   
   private void attemptToCreateSocketAndStreams()
   {
      try
      {
         if (DEBUG)
            System.out.println("!! Creating Socket on " + serverIPAddress +":" + serverPort);
         
         socket = new Socket(serverIPAddress, serverPort); 
         inputStream = socket.getInputStream();
         objectInputStream = new ObjectInputStream(inputStream);  
         
         connected.set(true);
      }
      catch (UnknownHostException e)
      {
         System.err.println("Unknown Host: " + serverIPAddress);
      }
      catch (IOException e)
      {
          System.err.println("Failed to create socket" + e.getMessage());
          e.printStackTrace();
      }
   }
   
   
   public void notifyConnectionBroke()
   {
      closeStreamsAndSockets();
      connected.set(false);
   }
   
   private void closeStreamsAndSockets()
   {
      try
      {
         if (objectInputStream != null)
            objectInputStream.close();
      }
      catch (IOException e)
      {
      }

      try
      {
         if (inputStream != null)
            inputStream.close();
      }
      catch (IOException e)
      {
      }

      try
      {
         if (socket != null)
            socket.close();
      }
      catch (IOException e)
      {
      }

      socket = null;
   }
   
   
   public void close()
   {
      keepOnTrying = false;
      
      closeStreamsAndSockets();
   }

   public boolean isConnected()
   {
      return connected.get();
      
      // Note: It would be nice to just be able to check this way below, but it doesn't work.
      // The only way you can know is if you experienced an exception when trying to read or write.
      // So the EstablishedAConnectionListener has to let you know...
      
//      // These two checks one would think would be enough, but no. 
//      // socket.isConnected always returns true once there is a connection.
//      if (socket == null) return false;
//      if (!socket.isConnected()) return false; 
//      if (socket.isClosed()) return false;
//      
//      if (objectInputStream == null) return false;
//
//      // To fully see if it is connected, you need to do something with it.
//      // Checking to see if there are bytes available should do the trick.
//      try
//      {
//         objectInputStream.available();
//      } 
//      catch (Exception e)
//      {
//         return false;
//      }
//      
//      return true;
   }
   
   private static void printIfDebug(String message)
   {
      if (DEBUG)
      {
         System.out.println(message);
      }
   }
}
