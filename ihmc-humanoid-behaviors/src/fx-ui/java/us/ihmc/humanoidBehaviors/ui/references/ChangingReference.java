package us.ihmc.humanoidBehaviors.ui.references;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An atomic reference with one frame of history to give it has changed
 * on the latest call to get().
 */
public class ChangingReference<T>
{
   private final AtomicReference<T> atomicReference;
   private T lastValue;
   private boolean changed = false;

   public ChangingReference(AtomicReference<T> atomicReference)
   {
      this.atomicReference = atomicReference;
      lastValue = atomicReference.get();
   }

   public T poll()
   {
      T newValue = atomicReference.get();

      if (newValue == lastValue)
         changed = false;
      else if (newValue == null)
         changed = true;
      else
         changed = !newValue.equals(lastValue);

      lastValue = newValue;
      return newValue;
   }

   public boolean hasChanged()
   {
      return changed;
   }
}
