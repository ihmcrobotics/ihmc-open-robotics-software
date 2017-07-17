package us.ihmc.robotDataLogger.dataBuffers;

import java.nio.ByteBuffer;

public class RegistrySendBufferSegment
{
   private static final double[] emptyJointStates = new double[0];

   
   private RegistrySendBuffer buffer;
   
   private final int offset;
   private final int length;
   private final boolean hasJointStates;
   
   public RegistrySendBufferSegment(int offset, int length, boolean hasJointStates)
   {
      this.offset = offset;
      this.length = length;
      this.hasJointStates = hasJointStates;
      
   }
   
   
   public void setBuffer(RegistrySendBuffer buffer)
   {
      this.buffer = buffer;
   }
   
   public ByteBuffer getBuffer()
   {
      ByteBuffer buffer = this.buffer.getBuffer();
      buffer.position(offset * 8);
      buffer.limit(buffer.position() + length * 8);
      return buffer;
   }

   public int getOffset()
   {
      return offset;
   }

   public long getUid()
   {
      return this.buffer.getUid();
   }


   public long getTransmitTime()
   {
      return buffer.getTransmitTime();
   }


   public long getTimestamp()
   {
      return buffer.getTimestamp();
   }


   public double[] getJointStates()
   {
      if(hasJointStates)
      {
         return buffer.getJointStates();
      }
      else
      {
         return emptyJointStates;
      }
   }


   public int getRegistryID()
   {
      return buffer.getRegistryID();
   }
   
   
   
}
