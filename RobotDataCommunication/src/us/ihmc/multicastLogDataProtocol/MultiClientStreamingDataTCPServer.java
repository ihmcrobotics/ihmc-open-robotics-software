package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.YoVariableHandShakeBuilder;

public class MultiClientStreamingDataTCPServer extends Thread
{

   private final ClientHandler handlers[] = new ClientHandler[4];
   private final ServerSocketChannel server;

   private ByteBuffer handshakeBuffer;

   public MultiClientStreamingDataTCPServer(int port, int dataSize, int bufferLength) throws IOException
   {
      this(port, ByteBuffer.allocateDirect(1), dataSize, bufferLength);
   }

   /**
    * Create new server 
    * @param port port to accept connections on
    * @param handshakeBuffer ByteBuffer with handshake data
    * @param dataSize Maximum length of data send by this server
    * @param bufferLength Number of buffers. Memory usage is 4 * bufferLength * dataSize bytes.
    *  
    * @throws IOException
    */
   public MultiClientStreamingDataTCPServer(int port, ByteBuffer handshakeBuffer, int dataSize, int bufferLength) throws IOException
   {
      server = ServerSocketChannel.open();
      server.bind(new InetSocketAddress(port));
      this.handshakeBuffer = handshakeBuffer;
      for (int i = 0; i < handlers.length; i++)
      {
         handlers[i] = new ClientHandler(dataSize, bufferLength);
         new Thread(handlers[i], "StreamingDataTCPServer-" + i).start();
      }
   }

   public MultiClientStreamingDataTCPServer(int port, YoVariableHandShakeBuilder handshakeBuilder, LogModelProvider logModelProvider, int dataSize, int bufferLength)
         throws IOException
   {
      this(port, createHandshakeBuffer(handshakeBuilder, logModelProvider), dataSize, bufferLength);
   }

   private static ByteBuffer createHandshakeBuffer(YoVariableHandShakeBuilder handshakeBuilder, LogModelProvider logModelProvider)
   {
      LogHandshake handshake = new LogHandshake();

      handshake.protoShake = handshakeBuilder.toByteArray();

      if (logModelProvider != null)
      {
         handshake.modelLoaderClass = logModelProvider.getLoader().getCanonicalName();
         handshake.modelName = logModelProvider.getModelName();
         handshake.model = logModelProvider.getModel();
         handshake.resourceDirectories = logModelProvider.getResourceDirectories();
         handshake.resourceZip = logModelProvider.getResourceZip();
      }

      return handshake.toBuffer();

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
            if (request == LogHandshake.HANDSHAKE_REQUEST)
            {
               handshakeBuffer.clear();
               while (handshakeBuffer.hasRemaining())
               {
                  client.write(handshakeBuffer);
               }
               client.close();
               continue;
            }
            else if (request == LogHandshake.STREAM_REQUEST)
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

      private final ReentrantLock dataLock = new ReentrantLock();
      private final Condition clientOnlineCondition = dataLock.newCondition();
      private final Condition dataAvailableCondition = dataLock.newCondition();

      private volatile boolean active = false;

      private long count;
      private int sendEveryNTicks;
      private SocketChannel client;

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
            dataLock.lock();
            while (!active)
            {
               try
               {
                  clientOnlineCondition.await();
               }
               catch (InterruptedException e)
               {
               }
            }

            // Flush queue
            data.poll();
            while (data.read() != null);
            data.flush();

            dataLock.unlock();
            
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
                  dataLock.lock();
                  try
                  {
                     dataAvailableCondition.await();
                  }
                  catch (InterruptedException e)
                  {
                  }
                  dataLock.unlock();
               }

            }
            System.out.println("Connection closed:  " + client);
            try
            {
               client.close();
            }
            catch (IOException e)
            {
            }

            dataLock.lock();
            active = false;
            dataLock.unlock();
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
         dataLock.lock();
         boolean internalActive = active;
         if (!internalActive)
         {
            this.client = client;
            this.sendEveryNTicks = sendEveryNTicks;
            this.active = true;
            clientOnlineCondition.signalAll();
         }
         dataLock.unlock();

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
               dataLock.lock();
               dataAvailableCondition.signalAll();
               dataLock.unlock();
            }
            count++;
         }
      }

   }

}
