package us.ihmc.tools;

public class SettableTimestampProvider implements TimestampProvider
{
   private long timestamp;
   
   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   @Override
   public long getTimestamp()
   {
      return timestamp;
   }

}
