package us.ihmc.tools.thread;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An atomic reference with one frame of history to give it has changed on
 * the latest call to get().
 * <p/>
 * The Activation/Activator classes are to set from another thread an object
 * which has a persistent value, for instance "activating" some functionality.
 * It is handy also that it provides access to if the value/activated-ness
 * just changed, to assist when state transition action is needed.
 */
public class ActivationReference<T>
{
   private final AtomicReference<T> atomicReference;
   private final T activatedValue;
   private boolean activationChanged = false;
   private T lastValue;

   public ActivationReference(AtomicReference<T> atomicReference, T activatedValue)
   {
      this.atomicReference = atomicReference;
      this.activatedValue = activatedValue;
      lastValue = atomicReference.get();
   }

   /**
    * Check if the current referenced value equals the active value, store if the activation changed
    * and store the t-1 value.
    *
    * @return current value equals activated value
    */
   public boolean poll()
   {
      T newValue = atomicReference.get();
      boolean newValueActivated = isValueActivated(newValue);
      boolean lastValueActivated = isValueActivated(lastValue);
      activationChanged = newValueActivated != lastValueActivated;
      lastValue = newValue;
      return newValueActivated;
   }

   public boolean peek()
   {
      T newValue = atomicReference.get();
      return isValueActivated(newValue);
   }

   /**
    * @return if the activation changed on the last call to {@link #poll()}
    */
   public boolean hasChanged()
   {
      return activationChanged;
   }

   /**
    * This thing provides null safety.
    */
   private boolean isValueActivated(T value)
   {
      boolean activated;
      if (value == activatedValue) // check reference equal or both null
         activated = true;
      else if (value == null)   // check value is null, knowing they aren't both null
         activated = false;
      else
         activated = value.equals(activatedValue);  // check the equals method, knowing neither is null
      return activated;
   }
}
