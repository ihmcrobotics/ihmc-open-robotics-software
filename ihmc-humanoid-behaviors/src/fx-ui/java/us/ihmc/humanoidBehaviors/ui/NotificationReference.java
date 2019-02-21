package us.ihmc.humanoidBehaviors.ui;

import java.util.concurrent.atomic.AtomicReference;

public class NotificationReference
{
   private final AtomicReference<Object> atomicReference;
   private Object polledValue;

   /**
    * Initialize with a new AtomicReference set to null.
    */
   public NotificationReference()
   {
      this.atomicReference = new AtomicReference<>();
      polledValue = null;
   }

   /**
    * Polls and clears the notification.
    *
    * @return if notification was present
    */
   public boolean poll()
   {
      polledValue = atomicReference.getAndSet(null);

      return polledValue != null;
   }

   /**
    * The initial or polled notification status.
    */
   public boolean read()
   {
      return polledValue != null;
   }

   /**
    * Sets the notification.
    */
   public void set()
   {
      atomicReference.set(new Object());
   }
}
