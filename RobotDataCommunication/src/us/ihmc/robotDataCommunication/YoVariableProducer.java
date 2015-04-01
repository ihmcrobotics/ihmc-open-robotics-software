package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.Collection;
import java.util.zip.CRC32;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.LogDataType;
import us.ihmc.multicastLogDataProtocol.SegmentedDatagramServer;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcaster;
import us.ihmc.multicastLogDataProtocol.control.LogControlServer;
import us.ihmc.utilities.compression.SnappyUtils;

public class YoVariableProducer extends Thread
{   
   private final ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final ConcurrentRingBuffer<RegistryBuffer>[] buffers;

   private final byte[] compressedBackingArray;
   private final ByteBuffer byteWriteBuffer;
   private final LongBuffer writeBuffer;
   private final ByteBuffer compressedBuffer;

   private final int jointStateOffset;
   
   private final LogSessionBroadcaster session;
   
   public YoVariableProducer(LogSessionBroadcaster session, LogControlServer controlServer, ConcurrentRingBuffer<FullStateBuffer> mainBuffer,
         Collection<ConcurrentRingBuffer<RegistryBuffer>> buffers)
   {
      super("YoVariableProducer");
      this.mainBuffer = mainBuffer;
      this.buffers = buffers.toArray(new ConcurrentRingBuffer[buffers.size()]);

      this.jointStateOffset = controlServer.getHandshakeBuilder().getNumberOfVariables();
      int numberOfJointStates = controlServer.getHandshakeBuilder().getNumberOfJointStates();
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;

      byteWriteBuffer = ByteBuffer.allocate(bufferSize);
      writeBuffer = byteWriteBuffer.asLongBuffer();

      compressedBackingArray = new byte[SnappyUtils.maxCompressedLength(bufferSize) + 4];
      compressedBuffer = ByteBuffer.wrap(compressedBackingArray);

      this.session = session;
   }

   private void updateBuffers(long timestamp)
   {
      for (int i = 0; i < buffers.length; i++)
      {
         ConcurrentRingBuffer<RegistryBuffer> buffer = buffers[i];
         if (buffer.poll())
         {
            RegistryBuffer newBuffer = buffer.peek();
            if (newBuffer != null && newBuffer.getTimestamp() < timestamp)
            {
               newBuffer = buffer.read();
               newBuffer.getIntoBuffer(writeBuffer, 1);
               buffer.flush();
            }
         }
      }
   }

   public void run()
   {
      
      SegmentedDatagramServer server;
      try
      {
         server = new SegmentedDatagramServer(session.getSessionID(), session.getInterface(), session.getGroup(), session.getPort());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      CRC32 crc32 = new CRC32();

      while (true)
      {
         if (mainBuffer.poll())
         {
            FullStateBuffer fullStateBuffer;

            while ((fullStateBuffer = mainBuffer.read()) != null)
            {
               
               writeBuffer.put(0, fullStateBuffer.getTimestamp());
               fullStateBuffer.getIntoBuffer(writeBuffer, 1);
               fullStateBuffer.getJointStatesInBuffer(writeBuffer, jointStateOffset + 1);
               updateBuffers(fullStateBuffer.getTimestamp());

               byteWriteBuffer.clear();
               compressedBuffer.clear();
               compressedBuffer.position(4);
               try
               {
                  SnappyUtils.compress(byteWriteBuffer, compressedBuffer);
                  compressedBuffer.flip();
               }
               catch (IllegalArgumentException | IOException e)
               {
                  e.printStackTrace();
                  continue;
               }

               crc32.reset();
               crc32.update(compressedBackingArray, 4, compressedBuffer.remaining() - 4);
               compressedBuffer.putInt(0, (int) crc32.getValue());
               
               try
               {
                  server.send(LogDataType.DATA, fullStateBuffer.getTimestamp(), compressedBuffer);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
            mainBuffer.flush();
         }
         try
         {
            Thread.sleep(1);
         }
         catch (InterruptedException e)
         {
            // Allow calling Thread.interrupt from YoVariableServer
            break;
         }
      }

      server.close();
   }

   public void close()
   {
      interrupt();
      try
      {
         join();
      }
      catch (InterruptedException e)
      {
      }
   }
}