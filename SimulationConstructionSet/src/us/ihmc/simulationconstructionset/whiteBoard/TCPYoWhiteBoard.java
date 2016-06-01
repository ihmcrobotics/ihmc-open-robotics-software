package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.ServerSocket;
import java.net.Socket;

import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class TCPYoWhiteBoard extends DataStreamYoWhiteBoard
{
   private static final boolean VERBOSE = true;

   private final String ipAddress;
   private int port;

   private ServerSocket serverSocket;
   private Socket tcpSocket;

   private boolean tcpThreadRunning = true;

   /**
    * Server constructor.
    */
   public TCPYoWhiteBoard(String name, final int port)
   {
      super(name, true, true);

      this.port = port;
      this.ipAddress = null;
   }

   /**
    * Client constructor.
    */
   public TCPYoWhiteBoard(String name, String ipAddress, int port)
   {
      super(name, true, true);

      this.ipAddress = ipAddress;
      this.port = port;
   }

   public void startTCPThread()
   {
      if (!tcpThreadRunning)
         throw new RuntimeException("Already running");

      tcpThreadRunning = false;

      ThreadTools.startAThread(new Runnable()
      {
         @Override
         public void run()
         {
            if (isAServer())
               runServer();
            else
               runClient();
         }
      }, getName() + "TCPThread");
   }

   private void runServer()
   {
      while (!tcpThreadRunning)
      {
         try
         {
            synchronized (getConnectionConch())
            {
               PrintTools.debug(VERBOSE, this, "runServer(): Accepting on port " + port);

               serverSocket = new ServerSocket(port);
               tcpSocket = serverSocket.accept();

               setupSocket();
            }

            setupAndConnect();
            runHandlingThread();

            closeYoWhiteBoard();
         }
         catch (IOException connectIOException)
         {
            PrintTools.error(this, connectIOException.getMessage());

            try
            {
               closeYoWhiteBoard();
            }
            catch (IOException closeIOException)
            {
               closeIOException.printStackTrace();
            }
         }

         ThreadTools.sleepSeconds(1.0);
      }
   }

   private void runClient()
   {
      while (!tcpThreadRunning)
      {
         try
         {
            synchronized (getConnectionConch())
            {
               PrintTools.debug(VERBOSE, this, "runClient(): Connecting to " + ipAddress + ":" + port);

               tcpSocket = new Socket(ipAddress, port);

               setupSocket();
            }
            
            setupAndConnect();
            runHandlingThread();

            closeYoWhiteBoard();
         }
         catch (ConnectException connectException)
         {
            PrintTools.error(this, "Failed to connect to " + ipAddress + ":" + port);
            PrintTools.error(this, connectException.getMessage());
         }
         catch (IOException connectIOException)
         {
            connectIOException.printStackTrace();

            try
            {
               closeYoWhiteBoard();
            }
            catch (IOException closeIOException)
            {
               closeIOException.printStackTrace();
            }
         }

         ThreadTools.sleepSeconds(1.0);
      }
   }

   private void setupSocket() throws IOException
   {
      tcpSocket.setTcpNoDelay(true);

      DataInputStream dataInputStream = new DataInputStream(new BufferedInputStream(tcpSocket.getInputStream()));
      DataOutputStream dataOutputStream = new DataOutputStream(new BufferedOutputStream(tcpSocket.getOutputStream()));

      PrintTools.debug(VERBOSE, this, "Connected to " + tcpSocket.getRemoteSocketAddress());

      super.setDataStreams(dataInputStream, dataOutputStream);
   }

   public boolean isTCPSocketConnected()
   {
      return tcpSocket != null && tcpSocket.isConnected();
   }

   @Override
   public void closeYoWhiteBoard() throws IOException
   {
      super.closeYoWhiteBoard();

      if (serverSocket != null)
         serverSocket.close();
      if (tcpSocket != null)
         tcpSocket.close();

      serverSocket = null;
      tcpSocket = null;
   }

   public void close() throws IOException
   {
      tcpThreadRunning = false;

      closeYoWhiteBoard();
   }

   @Override
   protected void allowThrowOutStalePacketsIfYouWish()
   {
      // Do nothing. TCP won't through out stale packets. Just UDP.
   }

   public boolean isAServer()
   {
      return ipAddress == null;
   }

   public int getPort()
   {
      return port;
   }
}
