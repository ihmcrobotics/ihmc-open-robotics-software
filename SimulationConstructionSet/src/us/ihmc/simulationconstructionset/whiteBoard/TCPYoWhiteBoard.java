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
   private Socket clientSocket;
   
   private boolean closed = true;
   
   public TCPYoWhiteBoard(String name, final int port)
   {      
      super(name, true, true);
      
      this.port = port;
      this.ipAddress = null;
   }

   public TCPYoWhiteBoard(String name, String ipAddress, int port)
   {      
      super(name, true, true);
      
      this.ipAddress = ipAddress;
      this.port = port;
   }
   
   public void startTCPThread()
   {
      if (!closed)
         throw new RuntimeException("Already started");
      
      closed = false;
      
      ThreadTools.startAThread(this, getName() + "TCPThread");
   }

   @Override
   public void run()
   {      
      if (isAServer())
         runServer();
      else
         runClient();
   }

   @Override
   public void whiteBoardSpecificConnect() throws IOException
   {
      super.whiteBoardSpecificConnect();
   }

   private void runServer()
   {
      while (!closed)
      {
         try
         {
            if (VERBOSE)
            {
               System.out.println("Waiting for server socket to accept");
            }
            
            serverSocket = new ServerSocket(port);
            // Get the port since if you use 0, Java will find one that is free
            this.port = serverSocket.getLocalPort();
            Socket socket = serverSocket.accept();
            socket.setTcpNoDelay(true);
            if (VERBOSE)
               System.out.println("Server socket accepted. Creating dataInputStream and dataOutputStream");

            DataInputStream dataInputStream = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
            DataOutputStream dataOutputStream = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));

            if (VERBOSE)
               System.out.println("Server all connected and running.");

            super.setDataStreams(dataInputStream, dataOutputStream);
            super.run();
            
            serverSocket.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void runClient()
   {
      while (!closed)
      {
         try
         {
            if (VERBOSE)
            {
               System.out.println("Trying to connect to server at " + ipAddress + " : " + port);
            }

            clientSocket = new Socket(ipAddress, port);
            clientSocket.setTcpNoDelay(true);

            DataInputStream dataInputStream = new DataInputStream(new BufferedInputStream(clientSocket.getInputStream()));
            DataOutputStream dataOutputStream = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));

            if (VERBOSE)
               System.out.println("Connected to server at " + ipAddress + " : " + port);

            super.setDataStreams(dataInputStream, dataOutputStream);
            super.run();
            
            clientSocket.close();
         }
         catch (ConnectException e)
         {
            PrintTools.error("Failed to connect to " + ipAddress + ":" + port);
            ThreadTools.sleep(500);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
   
   @Override
   public void whiteBoardSpecificClose() throws IOException
   {
      closed = true;
      
      if (serverSocket != null) serverSocket.close();
      if (clientSocket != null) clientSocket.close();
      
      serverSocket = null;
      clientSocket = null;
      
      super.whiteBoardSpecificClose();
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
