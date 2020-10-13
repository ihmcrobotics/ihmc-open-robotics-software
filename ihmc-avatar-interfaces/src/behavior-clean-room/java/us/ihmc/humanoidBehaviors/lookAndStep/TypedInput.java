package us.ihmc.humanoidBehaviors.lookAndStep;

import java.util.ArrayList;
import java.util.function.Consumer;

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
