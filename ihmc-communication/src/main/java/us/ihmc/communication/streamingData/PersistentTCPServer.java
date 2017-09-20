package us.ihmc.communication.streamingData;

import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class PersistentTCPServer
{
   private ServerSocket serverSocket = null;
   private final int port;
   private boolean isDoneClosing = false;
   private boolean allowingConnections = false;
   private final EstablishedAConnectionListener establishedAConnectionListener;

   private static final boolean DEBUG = false;

   public PersistentTCPServer(int port, EstablishedAConnectionListener establishedAConnectionListener)
   {
      this.port = port;
      this.establishedAConnectionListener = establishedAConnectionListener;
   }

   public void close()
   {
      allowingConnections = false;

      if (serverSocket != null)
      {
         try
         {
            serverSocket.close();
         }
         catch (IOException e)
         {
         }
      }
   }

   public void startOnAThread()
   {
      StreamingDataTCPServerRunner streamingDataTCPServerRunner = new StreamingDataTCPServerRunner();
      Thread thread = new Thread(streamingDataTCPServerRunner);
      thread.start();
   }

   public boolean isDoneClosing()
   {
      return isDoneClosing;
   }

   private class StreamingDataTCPServerRunner implements Runnable
   {

      public void run()
      {
         isDoneClosing = false;

         try
         {
            printIfDebug("creating server socket on port " + port);
            serverSocket = new ServerSocket(port);
            allowingConnections = true;
         }
         catch (IOException e1)
         {
            e1.printStackTrace();
         }

         while (allowingConnections && (serverSocket != null))
         {
            try
            {
               Socket socket = serverSocket.accept();
               if (DEBUG)
               {
                  System.out.println("received connection on " + port);
               }
               OutputStream outputStream = socket.getOutputStream();
               //TODO: Buffer them first, but somehow when I try this nothing works!
               //               BufferedOutputStream bufferedOutputStream = new BufferedOutputStream(outputStream);
               ObjectOutputStream objectOutputStream = new ObjectOutputStream(outputStream);

               establishedAConnectionListener.establishedAConnection(null, objectOutputStream);
            }
            catch (IOException e)
            {
            }
         }


         close();
         isDoneClosing = true;
      }
   }

   private static void printIfDebug(String message)
   {
      if (DEBUG)
      {
         System.out.println(message);
      }
   }
}
