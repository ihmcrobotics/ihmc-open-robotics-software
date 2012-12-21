package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;

public class FootstepConsumer implements FootstepProvider, StreamingDataConsumer
{
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final long dataIdentifier;

   public FootstepConsumer(long dataIdentifier)
   {
      this.dataIdentifier = dataIdentifier;
   }

   public boolean canHandle(Object object)
   {
      return object instanceof Footstep;
   }

   public void consume(long dataIdentifier, Object object)
   {
      if (dataIdentifier != this.dataIdentifier)
         throw new RuntimeException("Wrong data identifier: " + dataIdentifier + ". Expected: " + this.dataIdentifier);
      Footstep footstep = (Footstep) object;
      footstepQueue.add(footstep);
   }

   public Footstep poll()
   {
      return footstepQueue.poll();
   }

   public boolean isEmpty()
   {
      return footstepQueue.isEmpty();
   }

   public long getDataIdentifier()
   {
      return dataIdentifier;
   }
}
