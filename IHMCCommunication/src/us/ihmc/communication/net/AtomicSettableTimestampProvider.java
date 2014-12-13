package us.ihmc.communication.net;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.utilities.TimestampProvider;

public class AtomicSettableTimestampProvider implements TimestampProvider
{
   private AtomicLong atomicTimestamp = new AtomicLong();
   private final ArrayList<TimestampListener> listeners = new ArrayList<TimestampListener>();

   public long getTimestamp()
   {
      return atomicTimestamp.get();
   }
   
   public void increment(long increment)
   {
      notifyListeners(atomicTimestamp.addAndGet(increment));
   }
   
   public void set(long newTimestamp)
   {
      atomicTimestamp.set(newTimestamp);
      notifyListeners(newTimestamp);
   }
   
   public void attachListener(TimestampListener listener)
   {
      listeners.add(listener);
   }
   
   private void notifyListeners(long timestamp)
   {
      for(int i = 0; i < listeners.size(); i++)
      {
         listeners.get(i).timestampChanged(timestamp);
      }
   }
}
