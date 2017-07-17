package us.ihmc.robotDataLogger.dataBuffers;

public class RegistryBuffer implements Comparable<RegistryBuffer>
{

   protected int registryID;
   protected double[] jointStates;
   protected long timestamp;
   protected long transmitTime;
   protected int offset;
   
   public long getTransmitTime()
   {
      return transmitTime;
   }

   public void setTransmitTime(long transmitTime)
   {
      this.transmitTime = transmitTime;
   }

   protected long uid = 0;

   
   
   public int getOffset()
   {
      return offset;
   }

   public void setOffset(int offset)
   {
      this.offset = offset;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public long getUid()
   {
      return uid;
   }

   public double[] getJointStates()
   {
      return jointStates;
   }

   public void setUid(long uid)
   {
      this.uid = uid;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   @Override
   public int compareTo(RegistryBuffer o)
   {
      return Long.compare(this.timestamp, o.timestamp);
   }
   
   public int getRegistryID()
   {
      return registryID;
   }
   
   public void setRegistryID(int registryID)
   {
      this.registryID = registryID;
   }

}
