package us.ihmc.humanoidBehaviors.lookAndStep;

import java.util.ArrayList;
import java.util.function.Consumer;

public class TypedInput<T>
{
   private volatile T notification = null;
   private ArrayList<Consumer<T>> callbacks = new ArrayList<>();

   public synchronized T get()
   {
      return notification;
   }

   public void addCallback(Consumer<T> callback)
   {
      callbacks.add(callback);
   }

   public synchronized void set(T value)
   {
      notification = value;

      for (Consumer<T> callback : callbacks)
      {
         callback.accept(value);
      }
   }
}
