package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;

public class UDPTimestampServer
{
   public static final int TIMESTAMP_HEADER = 0x5d35bc23;
   
   private final Object lock = new Object();
   
   private final DatagramSocket serverSocket ;
   private final byte[] sendData = new byte[12];
   private final ByteBuffer sendDataBuffer = ByteBuffer.wrap(sendData);
   private final DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length);
   
   private InetAddress address;
   private int port;
   private boolean active = false;
   
   public UDPTimestampServer() throws SocketException
   {
      serverSocket = new DatagramSocket();
   }
   
   public void startSending(InetAddress target, int port)
   {
      synchronized (lock)
      {
         this.address = target;
         this.port = port;
         active = true;         
      }
   }
   
   
   public void sendTimestamp(long timestamp)
   {
      synchronized(lock)
      {
         if(active)
         {
            sendDataBuffer.putInt(0, TIMESTAMP_HEADER);
            sendDataBuffer.putLong(4, timestamp);
            sendPacket.setAddress(address);
            sendPacket.setPort(port);
            try
            {
               serverSocket.send(sendPacket);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
   
   public void close() 
   {
      synchronized(lock)
      {
         active = false;
         serverSocket.close();
      }
   }
}
