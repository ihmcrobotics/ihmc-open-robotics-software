package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.Collection;
import java.util.concurrent.TimeUnit;
import java.util.zip.CRC32;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.MultiClientStreamingDataTCPServer;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcaster;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.util.PeriodicThreadScheduler;

public class YoVariableProducer implements Runnable
{
   private static final int SEND_BUFFER_LENGTH = 1024;
   
   private final PeriodicThreadScheduler scheduler;
   
   private final ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final ConcurrentRingBuffer<RegistryBuffer>[] buffers;

   private final byte[] compressedBackingArray;
   private final ByteBuffer byteWriteBuffer;
   private final LongBuffer writeBuffer;
   private final ByteBuffer compressedBuffer;
   private final ByteBuffer compressedBufferDirect;

   private final int jointStateOffset;

   private final LogSessionBroadcaster session;
   private final YoVariableHandShakeBuilder handshakeBuilder;
   private final LogModelProvider logModelProvider;

   private MultiClientStreamingDataTCPServer server;
   
   private final LogDataHeader logDataHeader = new LogDataHeader();
   private final CRC32 crc32 = new CRC32();

   @SuppressWarnings("unchecked")
   public YoVariableProducer(PeriodicThreadScheduler scheduler, LogSessionBroadcaster session, YoVariableHandShakeBuilder handshakeBuilder, LogModelProvider logModelProvider,
         ConcurrentRingBuffer<FullStateBuffer> mainBuffer, Collection<ConcurrentRingBuffer<RegistryBuffer>> buffers)
   {
      this.scheduler = scheduler;
      this.mainBuffer = mainBuffer;
      this.handshakeBuilder = handshakeBuilder;
      this.logModelProvider = logModelProvider;
      this.buffers = buffers.toArray(new ConcurrentRingBuffer[buffers.size()]);

      this.jointStateOffset = handshakeBuilder.getNumberOfVariables();
      int numberOfJointStates = handshakeBuilder.getNumberOfJointStates();
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

   public void start()
   {
      try
      {
         // Make server here, so it is open before the logger connects
         server = new MultiClientStreamingDataTCPServer(session.getPort(), handshakeBuilder, logModelProvider, compressedBackingArray.length, SEND_BUFFER_LENGTH);
         server.start();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
   }

   public void run()
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
            logDataHeader.setUid(fullStateBuffer.getUid());
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

   }

   public void close()
   {
      scheduler.shutdown();
      server.close();
      
   }
}