package us.ihmc.behaviors.lookAndStep;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * An typed input to a module as in the subsumption style architecture.
 */
public class TypedInput<T>
{
   private volatile T latestData = null;
   private ArrayList<Consumer<T>> callbacks = new ArrayList<>();

   public synchronized T getLatest()
   {
      return latestData;
   }

   public void addCallback(Consumer<T> callback)
   {
      callbacks.add(callback);
   }

   public synchronized void set(T value)
   {
      latestData = value;

      for (Consumer<T> callback : callbacks)
      {
         callback.accept(value);
      }
   }
}
