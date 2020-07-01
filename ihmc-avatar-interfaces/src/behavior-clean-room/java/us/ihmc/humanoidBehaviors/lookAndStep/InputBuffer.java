package us.ihmc.humanoidBehaviors.lookAndStep;

/**
 * This class is hostile to null data. It will throw RuntimeExceptions if null data is passed in
 * and before returning null.
 *
 * @param <T>
 */
public class InputBuffer<T>
{
   private volatile T data;
   private volatile T stored;

   public synchronized void accept(T data)
   {
      this.data = data;
   }

   public synchronized void store()
   {
      if (data == null)
      {
         throw new RuntimeException("Cannot store null data. Please call accept first with non-null value.");
      }
      else
      {
         stored = data;
      }
   }

   public synchronized T getStored()
   {
      if (stored == null)
      {
         throw new RuntimeException("Please store a value first.");
      }
      else
      {
         return stored;
      }
   }

   public synchronized T getLatest()
   {
      if (data == null)
      {
         throw new RuntimeException("Please call accept with data first.");
      }
      else
      {
         return data;
      }
   }
}
