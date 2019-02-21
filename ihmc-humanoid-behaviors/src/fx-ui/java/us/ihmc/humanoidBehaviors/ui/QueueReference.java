package us.ihmc.humanoidBehaviors.ui;

import java.util.concurrent.atomic.AtomicReference;

public class QueueReference<T>
{
   private final AtomicReference<T> atomicReference;
   private T polledValue;

   /**
    * Initialize with a new AtomicReference set to null.
    */
   public QueueReference()
   {
      this.atomicReference = new AtomicReference<>(null);
      polledValue = null;
   }

   /**
    * Get the atomic value, store it for a later call to read, and return if it was null.
    *
    * @return value available
    */
   public boolean poll()
   {
      polledValue = atomicReference.getAndSet(null);

      return polledValue != null;
   }

   /**
    * If the initial or polled value was not null.
    *
    * @return polled value was not null
    */
   public boolean hasNext()
   {
      return polledValue != null;
   }

   /**
    * The initial or polled value.
    *
    * @return polled value
    */
   public T read()
   {
      return polledValue;
   }

   /**
    * Submits a value to the queue.
    *
    * @param value
    */
   public void add(T value)
   {
      atomicReference.set(value);
   }
}
