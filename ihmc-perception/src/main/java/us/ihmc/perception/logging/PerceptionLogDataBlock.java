package us.ihmc.perception.logging;

public abstract class PerceptionLogDataBlock
{
   static final int MAX_BYTE_BUFFER_SIZE = 1000000;
   static final int MAX_FLOAT_BUFFER_SIZE = 100000;

   private String topicName;
   private long timeOfReception;

   public String getTopicName()
   {
      return topicName;
   }

   public void setTopicName(String topicName)
   {
      this.topicName = topicName;
   }

   public long getTimeOfReception()
   {
      return timeOfReception;
   }

   public void setTimeOfReception(long timeOfReception)
   {
      this.timeOfReception = timeOfReception;
   }
}
