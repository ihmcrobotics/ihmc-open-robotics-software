package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.Collection;
import java.util.zip.CRC32;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.SingleThreadMultiClientStreamingDataTCPServer;
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
   private final ByteBuffer compressedBufferDirect;

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

      compressedBackingArray = new byte[SnappyUtils.maxCompressedLength(bufferSize) + LogDataHeader.length()];
      compressedBuffer = ByteBuffer.wrap(compressedBackingArray);
      compressedBufferDirect = ByteBuffer.allocateDirect(compressedBuffer.capacity());
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
      
      SingleThreadMultiClientStreamingDataTCPServer server;
      try
      {
         server = new SingleThreadMultiClientStreamingDataTCPServer(session.getPort());
         server.start();
//               new SegmentedDatagramServer(session.getSessionID(), session.getInterface(), session.getGroup(), session.getPort());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      LogDataHeader logDataHeader = new LogDataHeader();
      CRC32 crc32 = new CRC32();
      long uid = 0;
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
               compressedBuffer.position(LogDataHeader.length());
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
               int dataSize = compressedBuffer.remaining() - LogDataHeader.length();
               crc32.update(compressedBackingArray, LogDataHeader.length() + compressedBuffer.arrayOffset(), dataSize);
               logDataHeader.setUid(++uid);
               logDataHeader.setTimestamp(fullStateBuffer.getTimestamp());
               logDataHeader.setDataSize(dataSize);
               logDataHeader.setCrc32((int) crc32.getValue());
               logDataHeader.writeBuffer(0, compressedBuffer);
               compressedBufferDirect.clear();
               compressedBufferDirect.put(compressedBuffer);
               compressedBufferDirect.flip();
               server.send(compressedBufferDirect);
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