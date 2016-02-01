package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

public class TCPYoWhiteBoard extends DataStreamYoWhiteBoard
{
   private static final boolean VERBOSE = false;

   private final String ipAddress;
   private final int port;

   private ServerSocket serverSocket;
   private Socket clientSocket;
   
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

   public void run()
   {
      if (ipAddress == null)
         runServer();
      else
         runClient();
   }

   private void runServer()
   {
      try
      {
         serverSocket = new ServerSocket(port);

         if (VERBOSE)
            System.out.println("Waiting for server socket to accept");

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
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }



   private void runClient()
   {
      if (VERBOSE)
         System.out.println("Trying to connect to server at " + ipAddress + " : " + port);

      try
      {
         clientSocket = new Socket(ipAddress, port);

         clientSocket.setTcpNoDelay(true);

         DataInputStream dataInputStream = new DataInputStream(new BufferedInputStream(clientSocket.getInputStream()));
         DataOutputStream dataOutputStream = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));

         if (VERBOSE)
            System.out.println("Connected to server at " + ipAddress + " : " + port);

         super.setDataStreams(dataInputStream, dataOutputStream);
         super.run();
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public void whiteBoardSpecificClose() throws IOException
   {
      if (serverSocket != null) serverSocket.close();
      if (clientSocket != null) clientSocket.close();
      
      serverSocket = null;
      clientSocket = null;
      
      super.whiteBoardSpecificClose();
   }

   protected void allowThrowOutStalePacketsIfYouWish()
   {
      // Do nothing. TCP won't through out stale packets. Just UDP.
   }
}
