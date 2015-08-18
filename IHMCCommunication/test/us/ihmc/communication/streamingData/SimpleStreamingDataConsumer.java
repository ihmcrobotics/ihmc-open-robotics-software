package us.ihmc.communication.streamingData;




public class SimpleStreamingDataConsumer implements StreamingDataConsumer
{
   private final long dataIdentifier;
   
   private int largestIndexSeen = 0;
   
   public SimpleStreamingDataConsumer()
   {
      this(SimpleStreamedData.getSerialVersionUID());
   }
   
   public SimpleStreamingDataConsumer(long dataIdentifier)
   {
      this.dataIdentifier = dataIdentifier;
   }
   
   public void consume(long dataIdentifier, Object dataObject)
   {     
      SimpleStreamedData simpleStreamedData = (SimpleStreamedData) dataObject;
      
      int index = simpleStreamedData.getValue();
//      System.out.println("Consuming SimpleStreamedData: " + index);
      if (index > largestIndexSeen)
      {
         largestIndexSeen = index;
      }
   }

   public int getLargestIndexSeen()
   {
      return largestIndexSeen;
   }

   public boolean canHandle(Object object)
   {
      return (object instanceof SimpleStreamedData);
   }

   public long getDataIdentifier()
   {
      return dataIdentifier;
   }
  
}

