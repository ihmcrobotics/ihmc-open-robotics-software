package us.ihmc.communication.remote;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * <p>Title: TCPNetworkNode </p>
 *
 * <p>Description: Represents either a simple client or server, provides utilities for connecting over tcp to other nodes (or anything else that uses tcp),
 * and creates streams for communicating between nodes.</p>
 *
 * @author jrebula
 */
public class TCPNetworkNode
{
   @SuppressWarnings("unused")
   private boolean isServer;
   protected DataOutputStream dataOutputStream;
   protected DataInputStream dataInputStream;

   protected Socket socket;
   protected ServerSocket serverSocket;

   private TCPNetworkNode(boolean isServer)
   {
      this.isServer = isServer;
   }

   protected TCPNetworkNode()
   {
   }

   public TCPNetworkNode(ServerSocket serverSocket, Socket socket)
   {
      this.isServer = true;
      this.serverSocket = serverSocket;
      this.socket = socket;
   }

   public static TCPNetworkNode startServer(int port) throws IOException
   {
      TCPNetworkNode ret = new TCPNetworkNode(true);
      ret.serverSocket = new ServerSocket(port);
      ret.socket = ret.serverSocket.accept();

//    System.out.println("ret.socket.getInetAddress()=" + ret.socket.getInetAddress());
      return ret;
   }

   public boolean isConnected()
   {
      return socket.isConnected();
   }

   public static TCPNetworkNode createClientConnectedToServerAt(String ip, int port) throws IOException
   {
      TCPNetworkNode ret = new TCPNetworkNode(false);
      ret.serverSocket = null;
      ret.socket = new Socket(ip, port);

      return ret;
   }

   public void close() throws IOException
   {
      if (dataInputStream != null)
      {
         dataInputStream.close();
      }

      if (dataOutputStream != null)
      {
         dataOutputStream.flush();
         dataOutputStream.close();
      }

      if (serverSocket != null)
      {
         serverSocket.close();
      }

      if (socket != null)
      {
         socket.close();
      }
   }

   public DataInputStream getDataInputStream() throws IOException
   {
      if (dataInputStream == null)
      {
         dataInputStream = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
      }

      return dataInputStream;
   }

   public DataOutputStream getDataOutputStream() throws IOException
   {
      if (dataOutputStream == null)
      {
         dataOutputStream = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));
      }

      return dataOutputStream;
   }

}
