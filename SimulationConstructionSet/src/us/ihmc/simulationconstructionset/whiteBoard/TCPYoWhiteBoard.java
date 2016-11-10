package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class TCPYoWhiteBoard extends DataStreamYoWhiteBoard
{
	private static final boolean VERBOSE = false;
	private static final boolean PRINT_ERRORS = false;
	private static final long THREAD_PERIOD = 1L;
	private static final TimeUnit TIME_UNIT = TimeUnit.SECONDS;

   private final ScheduledExecutorService executorService;
	private ScheduledFuture<?> activeScheduled = null;

   private final String ipAddress;
   private int port;

   private ServerSocket serverSocket;
   private Socket tcpSocket;

   /**
    * Server constructor.
    */
   public TCPYoWhiteBoard(String name, final int port)
   {
      super(name, true, true);

      this.port = port;
      this.ipAddress = null;
      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name + "TCPServerThread");
      executorService = Executors.newScheduledThreadPool(1, threadFactory);
   }

   /**
    * Client constructor.
    */
   public TCPYoWhiteBoard(String name, String ipAddress, int port)
   {
      super(name, true, true);

      this.ipAddress = ipAddress;
      this.port = port;
      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name + "TCPClientThread");
      executorService = Executors.newScheduledThreadPool(1, threadFactory);
   }

   public void startTCPThread()
   {
      if (activeScheduled != null)
         throw new RuntimeException("Already running");

      Runnable command;
      if (isAServer())
         command = createServerRunnable();
      else
         command = createClientRunnable();
      activeScheduled = executorService.scheduleAtFixedRate(command, 0, THREAD_PERIOD, TIME_UNIT);
   }

   private Runnable createServerRunnable()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               doActionAsServer();
            }
            catch (Exception e)
            {
               if (PRINT_ERRORS)
                  e.printStackTrace();
            }
         }
      };
   }

   private Runnable createClientRunnable()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               doActionAsClient();
            }
            catch (Exception e)
            {
               if (PRINT_ERRORS)
                  e.printStackTrace();
            }
         }
      };
   }

   private void doActionAsServer()
   {
      try
      {
         synchronized (getConnectionConch())
         {
            if (VERBOSE)
               PrintTools.debug(this, "runServer(): Accepting on port " + port);

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
         if (PRINT_ERRORS)
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
   }

   private void doActionAsClient()
   {
      try
      {
         synchronized (getConnectionConch())
         {
            if (VERBOSE)
               PrintTools.debug(this, "runClient(): Connecting to " + ipAddress + ":" + port);

            tcpSocket = new Socket(ipAddress, port);

            setupSocket();
         }

         setupAndConnect();
         runHandlingThread();

         closeYoWhiteBoard();
      }
      catch (ConnectException connectException)
      {
         if (PRINT_ERRORS)
         {
            PrintTools.error(this, "Failed to connect to " + ipAddress + ":" + port);
            PrintTools.error(this, connectException.getMessage());
         }
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
   }

   private void setupSocket() throws IOException
   {
      tcpSocket.setTcpNoDelay(true);

      DataInputStream dataInputStream = new DataInputStream(new BufferedInputStream(tcpSocket.getInputStream()));
      DataOutputStream dataOutputStream = new DataOutputStream(new BufferedOutputStream(tcpSocket.getOutputStream()));

      if (VERBOSE)
         PrintTools.debug(this, "Connected to " + tcpSocket.getRemoteSocketAddress());

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
      if (activeScheduled != null)
      {
         activeScheduled.cancel(true);
         activeScheduled = null;
      }
      executorService.shutdownNow();
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
