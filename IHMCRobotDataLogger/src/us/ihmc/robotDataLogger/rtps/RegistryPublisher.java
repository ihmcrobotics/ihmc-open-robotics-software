package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotDataLogger.FullStateBuffer;
import us.ihmc.robotDataLogger.LogDataHeader;
import us.ihmc.robotDataLogger.RegistryBuffer;
import us.ihmc.tools.compression.SnappyUtils;

public class RegistryPublisher
{
   private static final int BUFFER_CAPACITY = 128;
   
   private final int registryID;
   private long uid = 0;
   private final ConcurrentRingBuffer<RegistryBuffer> ringBuffer;
   
   public RegistryPublisher(int registryID, us.ihmc.concurrent.Builder<RegistryBuffer> builder)
   {
      this.registryID = registryID;
      this.ringBuffer = new ConcurrentRingBuffer<>(builder, BUFFER_CAPACITY);
   }
   
   
   void update(long timestamp)
   {
      RegistryBuffer buffer = ringBuffer.next();
      if(buffer != null)
      {
         buffer.update(timestamp, uid);
         ringBuffer.commit();
      }
      
      uid++;
   }
   
   
   private class VariableUpdateThread implements Runnable
   {
      private VariableUpdateThread()
      {
         
      }

      @Override
      public void run()
      {
         while (ringBuffer.poll())
         {
            RegistryBuffer fullStateBuffer;

            if ((fullStateBuffer = ringBuffer.read()) != null)
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
               logDataHeader.setType(LogDataHeader.DATA_PACKET);
               logDataHeader.setDataSize(dataSize);
               logDataHeader.setCrc32((int) crc32.getValue());
               logDataHeader.writeBuffer(0, compressedBuffer);
               compressedBufferDirect.clear();
               compressedBufferDirect.put(compressedBuffer);
               compressedBufferDirect.flip();
               server.send(compressedBufferDirect);
               
               keepAliveCounter = 0;
            }
            ringBuffer.flush();
         }
            
      }
      
      
   }
   
}
