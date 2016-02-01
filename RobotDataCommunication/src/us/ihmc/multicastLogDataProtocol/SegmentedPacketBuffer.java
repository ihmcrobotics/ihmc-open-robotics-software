package us.ihmc.multicastLogDataProtocol;

import java.nio.ByteBuffer;

public class SegmentedPacketBuffer
{
   private final LogDataType dataType;
   private final long uid;
   private final long timestamp;
   
   private final ByteBuffer[] dataBuffer;
   private boolean complete = false;
   
   
   public SegmentedPacketBuffer(SegmentHeader header)
   {
      this.dataType = header.getType();
      this.uid = header.getPackageID();
      this.timestamp = header.getTimestamp();
      dataBuffer = new ByteBuffer[header.getSegmentCount()];
   }
   
   public void addSegment(int segmentID, ByteBuffer data)
   {
      if(segmentID < dataBuffer.length)
      {
         dataBuffer[segmentID] = ByteBuffer.allocate(data.remaining());
         dataBuffer[segmentID].put(data);
         dataBuffer[segmentID].flip();
         
         boolean complete = true;
         for(int i = 0; i < dataBuffer.length; i++)
         {
            if(dataBuffer[i] == null)
            {
               complete = false;
            }
         }
         this.complete = complete;
      }
   }
   
   public ByteBuffer getBuffer()
   {
      int size = 0;
      for(int i = 0; i < dataBuffer.length; i++)
      {
         size += dataBuffer[i].remaining();
      }
      
      ByteBuffer buf = ByteBuffer.allocate(size);
      
      for(int i = 0; i < dataBuffer.length; i++)
      {
         buf.put(dataBuffer[i]);
      }
      buf.flip();
      return buf;
   }
   
   public boolean isComplete()
   {
      return complete;
   }

   public LogDataType getDataType()
   {
      return dataType;
   }

   public long getTimestamp()
   {
      return timestamp;
   }
   
   public long getUid()
   {
      return uid;
   }
}
