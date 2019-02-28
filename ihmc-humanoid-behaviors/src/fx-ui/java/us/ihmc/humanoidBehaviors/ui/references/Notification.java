package us.ihmc.humanoidBehaviors.ui.references;

public class Notification
{
   private boolean notification = false;
   private boolean previousValue = false;

   /**
    * Polls and clears the notification.
    *
    * @return if notification was present
    */
   public boolean poll()
   {
      previousValue = notification;
      notification = false;
      return previousValue;
   }

   /**
    * The initial or polled notification status.
    */
   public boolean read()
   {
      return previousValue;
   }

   /**
    * Sets the notification.
    */
   public void set()
   {
      notification = true;
   }
}
