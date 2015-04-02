package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.channels.ClosedByInterruptException;
import java.nio.channels.SocketChannel;

import us.ihmc.robotDataCommunication.LogDataHeader;

public class StreamingDataTCPClient extends Thread
{
   private volatile boolean running = false;

   private final InetSocketAddress address;
   private final LogPacketHandler updateHandler;

   public StreamingDataTCPClient(InetAddress dataIP, int port, LogPacketHandler updateHandler)
   {
      this.address = new InetSocketAddress(dataIP, port);
      this.updateHandler = updateHandler; 
   }

   @Override
   public void run()
   {
      running = true;
      
      SocketChannel connection;
      try
      {
         connection = SocketChannel.open();
         connection.connect(address);

//         connection.socket().setReceiveBufferSize(1000000);
//         connection.socket().setKeepAlive(true);
//         connection.socket().setTcpNoDelay(true);
         connection.socket().setSoTimeout(20000);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      ByteBuffer headerBuffer = ByteBuffer.allocate(LogDataHeader.length());
      
      DATALOOP:
      while (running)
      {
         try
         {
            headerBuffer.clear();
            int read = connection.read(headerBuffer);
            if(read == -1)
            {
               updateHandler.timeout();
               break DATALOOP;
            }
            headerBuffer.clear();
            LogDataHeader header = new LogDataHeader();
            if(!header.readBuffer(headerBuffer))
            {
//               System.err.println("Expected header, got data. Scanning till new header found.");
               continue DATALOOP; // Cannot read buffer, continue with data loop hopefully latching on to the data stream again
            }
            updateHandler.timestampReceived(header.getTimestamp());
            
            ByteBuffer dataBuffer = ByteBuffer.allocate(header.getDataSize());
           
            while(dataBuffer.hasRemaining())
            {
               read = connection.read(dataBuffer);
               if(read == -1)
               {
                  updateHandler.timeout();
                  break DATALOOP;
               }
            }
            
            dataBuffer.flip();
            updateHandler.newDataAvailable(header, dataBuffer);
            
            
         }
         catch (ClosedByInterruptException e)
         {
            updateHandler.timeout();
            break DATALOOP;
         }
         catch (SocketTimeoutException e)
         {
            updateHandler.timeout();
            break DATALOOP;
         }
         catch (IOException e)
         {
            e.printStackTrace();
            break DATALOOP;
         }
         
      }
      running = false;
   }

   public boolean isRunning()
   {
      return running;
   }

   public void close()
   {
      running = false;
      interrupt();
   }
}
