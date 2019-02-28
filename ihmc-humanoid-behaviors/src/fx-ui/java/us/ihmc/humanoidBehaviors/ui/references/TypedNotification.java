package us.ihmc.humanoidBehaviors.ui.references;

public class TypedNotification<T>
{
   private T notification = null;
   private T previousValue = null;

   /**
    * Get the atomic value, store it for a later call to read, and return if it was null.
    *
    * @return value available
    */
   public boolean poll()
   {
      previousValue = notification;
      notification = null;
      return previousValue != null;
   }

   /**
    * If the initial or polled value was not null.
    *
    * @return polled value was not null
    */
   public boolean hasNext()
   {
      return previousValue != null;
   }

   /**
    * The initial or polled value.
    *
    * @return polled value
    */
   public T read()
   {
      return previousValue;
   }

   /**
    * Submits a value to the queue.
    *
    * @param value
    */
   public void add(T value)
   {
      notification = value;
   }
}
