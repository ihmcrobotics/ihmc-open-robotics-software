package us.ihmc.behaviors.activeMapping;

import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;

import java.net.InetSocketAddress;
import java.util.Scanner;

class ContinuousHikingCommand
{
   public ... create the feidls for the gson node...
}

public class ContinuousHikingWebSocketServer extends WebSocketServer
{

   public ContinuousHikingWebSocketServer(int port)
   {
      super(new InetSocketAddress(port));
   }

   @Override
   public void onOpen(WebSocket conn, ClientHandshake handshake)
   {
      System.out.println("Client connected: " + conn.getRemoteSocketAddress());
   }

   @Override
   public void onClose(WebSocket conn, int code, String reason, boolean remote)
   {
      System.out.println("Client disconnected: " + conn.getRemoteSocketAddress());
   }

   @Override
   public void onMessage(WebSocket conn, String message)
   {
      System.out.println("Received command: " + message);
      conn.send("ACK: " + message);
   }

   @Override
   public void onError(WebSocket conn, Exception ex)
   {
      ex.printStackTrace();
   }

   @Override
   public void onStart()
   {

   }

   public static void main(String[] args)
   {
      int port = 8765;
      ContinuousHikingWebSocketServer server = new ContinuousHikingWebSocketServer(port);
      server.start();
      System.out.println("WebSocket server started on port " + port);
      System.out.println("Press Enter to stop the server...");
      new Scanner(System.in).nextLine(); // wait for Enter
      try
      {
         server.stop();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
}
