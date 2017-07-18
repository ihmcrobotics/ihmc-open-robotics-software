package us.ihmc.robotDataLogger.dataBuffers;

public class RegistryBuffer implements Comparable<RegistryBuffer>
{

   protected int registryID;
   protected long timestamp;
   protected long transmitTime;
   protected int offset;
   protected int numberOfVariables;
   
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
   
   

   public int getNumberOfVariables()
   {
      return numberOfVariables;
   }

   public void setNumberOfVariables(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public long getUid()
   {
      return uid;
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
      return (this.timestamp < o.timestamp) ? -1 : ((this.timestamp == o.timestamp) ? Long.compare(this.uid, o.uid) : 1);
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
