package us.ihmc.communication.subscribers;

import us.ihmc.commons.thread.TypedNotification;

import java.util.function.BiFunction;

public class FilteredNotification<T> extends TypedNotification<T>
{
   private final BiFunction<T, T, Boolean> acceptanceFunction;

   /**
    * @param acceptanceFunction the previous and new values in, and acceptance out
    *                           Should handle null for both previous and new
    */
   public FilteredNotification(BiFunction<T, T, Boolean> acceptanceFunction)
   {
      this.acceptanceFunction = acceptanceFunction;
   }

   public boolean pollFiltered()
   {
      if (peekHasValue()) // Poll the new value
      {
         T previousValue = read(); // May be null
         poll();
         T newValue = read(); // Shouldn't be null unless something called set(null)
         return acceptanceFunction.apply(previousValue, newValue);
      }
      else
      {
         return false;
      }
   }
}
