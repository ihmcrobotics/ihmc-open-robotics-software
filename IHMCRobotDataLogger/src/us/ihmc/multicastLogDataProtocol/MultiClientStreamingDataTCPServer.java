package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;

public class MultiClientStreamingDataTCPServer extends Thread
{

   public final static byte STREAM_REQUEST = 0x7A;
   
   private final ClientHandler handlers[] = new ClientHandler[4];
   private final ServerSocketChannel server;


   /**
    * Create new server 
    * @param port port to accept connections on
    * @param handshakeBuffer ByteBuffer with handshake data
    * @param dataSize Maximum length of data send by this server
    * @param bufferLength Number of buffers. Memory usage is 4 * bufferLength * dataSize bytes.
    *  
    * @throws IOException
    */
   public MultiClientStreamingDataTCPServer(int port, int dataSize, int bufferLength) throws IOException
   {
      server = ServerSocketChannel.open();
      server.bind(new InetSocketAddress(port));
      for (int i = 0; i < handlers.length; i++)
      {
         handlers[i] = new ClientHandler(dataSize, bufferLength);
         new Thread(handlers[i], "StreamingDataTCPServer-" + i).start();
      }
   }

   public void send(ByteBuffer data)
   {
      for (int i = 0; i < handlers.length; i++)
      {
         ClientHandler handler = handlers[i];
         data.mark();
         handler.send(data);
         data.reset();
      }
   }

   private void addClient(int sendPacketsEveryNthTick, SocketChannel client)
   {
      for(int i = 0; i < handlers.length; i++)
      {
         if(handlers[i].setClient(client, sendPacketsEveryNthTick))
         {
            return;
         }
      }
      // To many connections, closing 
      try
      {
         System.out.println("Too many connections, closing client " + client);
         client.close();
      }
      catch (IOException e)
      {

      }
   }

   public void run()
   {
      ByteBuffer startByteBuffer = ByteBuffer.allocateDirect(1);
      for (;;)
      {
         try
         {
            SocketChannel client = server.accept();
            client.socket().setSendBufferSize(1000000); // Large send buffer to not loose packets

            startByteBuffer.clear();
            int read = client.read(startByteBuffer); // Reading one byte is guaranteed
            startByteBuffer.flip();
            if (read == -1)
            {
               client.close();
               continue;
            }

            byte request = startByteBuffer.get();
            if (request == STREAM_REQUEST)
            {
               startByteBuffer.clear();
               read = client.read(startByteBuffer);
               startByteBuffer.flip();
               if (read == -1)
               {
                  client.close();
                  continue;
               }
               addClient(startByteBuffer.get() & 0xFF, client);
            }
            else
            {
               System.err.println(client + " send unknown request");
            }

         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public void close()
   {
      try
      {
         server.close();
      }
      catch (IOException e)
      {
      }
   }

   private static class ClientHandler implements Runnable
   {

      private final ConcurrentRingBuffer<ByteBuffer> data;


      private final Object dataLock = new Object();
      
      private volatile boolean active = false;

      private long count;
      private int sendEveryNTicks;
      private SocketChannel client;
      

      private AtomicReference<InetSocketAddress> udpAddress = new AtomicReference<InetSocketAddress>(null);
      
      private ClientHandler(final int dataSize, int bufferLength)
      {
         data = new ConcurrentRingBuffer<>(new Builder<ByteBuffer>()
         {

            @Override
            public ByteBuffer newInstance()
            {
               return ByteBuffer.allocateDirect(dataSize);
            }
         }, bufferLength);
      }

      @Override
      public void run()
      {
         for (;;)
         {
            synchronized (dataLock)
            {
               while (!active)
               {
                  try
                  {
                     dataLock.wait();
                  }
                  catch (InterruptedException e)
                  {
                  }
               }
               
               // Flush queue
               data.poll();
               while (data.read() != null);
               data.flush();
               
            }            
            System.out.println("Accepted client:  " + client);

            for (;;)
            {
               if (!client.isConnected())
               {
                  if (!client.isConnectionPending())
                  {
                     break;
                  }
                  continue;
               }

               if (data.poll())
               {
                  ByteBuffer currentData = data.read();
                  try
                  {
                     client.write(currentData);
                  }
                  catch (IOException e)
                  {
                     break;
                  }
                  data.flush();
               }
               else
               {
                  synchronized (dataLock)
                  {
                     try
                     {
                        dataLock.wait();
                     }
                     catch (InterruptedException e)
                     {
                     }
                     
                  }
               }

            }
            System.out.println("Connection closed:  " + client);
            
            udpAddress.set(null);
            try
            {
               client.close();
            }
            catch (IOException e)
            {
            }

            synchronized (dataLock)
            {
               active = false;               
            }
         }
      }

      /**
       * Set the client
       * @param client
       * @param sendEveryNTicks
       * 
       * @return true if this thread can accept the client
       */
      public boolean setClient(SocketChannel client, int sendEveryNTicks)
      {
         boolean internalActive;
         synchronized (dataLock)
         {
            internalActive = active;
            if (!internalActive)
            {
               this.client = client;
               this.sendEveryNTicks = sendEveryNTicks;
               try
               {
                  SocketAddress socketAddress = client.getRemoteAddress();
                  if(socketAddress instanceof InetSocketAddress)
                  {
                     this.udpAddress.set(new InetSocketAddress(((InetSocketAddress) socketAddress).getAddress(), ((InetSocketAddress) socketAddress).getPort()));
                  }
                  else
                  {
                     this.udpAddress = null;
                  }
               }
               catch(Exception e)
               {
                  this.udpAddress.set(null);
               }
               this.active = true;
               dataLock.notifyAll();
            }
            
         }
         return !internalActive;
      }

      public void send(ByteBuffer dataToSend)
      {
         if (active)
         {

            if (count % sendEveryNTicks == 0)
            {
               ByteBuffer nextData = data.next();

               if (nextData != null)
               {
                  nextData.clear();
                  nextData.put(dataToSend);
                  nextData.flip();
               }
               data.commit();
               synchronized (dataLock)
               {
                  dataLock.notifyAll();
               }
            }
            count++;
         }
      }
      
      public InetSocketAddress getUDPAddress()
      {
         return udpAddress.get();
      }

   }
   
   public InetSocketAddress getUDPAddress(int client)
   {
      return handlers[client].getUDPAddress();
   }

   public int getMaximumNumberOfConnections()
   {
      return handlers.length;
   }

}
