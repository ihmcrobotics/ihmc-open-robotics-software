package us.ihmc.behaviors.lookAndStep;

import us.ihmc.commons.thread.Notification;

import java.util.ArrayList;

/**
 * An input to a module as in the subsumption style architecture.
 */
public class Input
{
   private final Notification notification = new Notification();
   private final ArrayList<Runnable> callbacks = new ArrayList<>();

   public Notification getNotification()
   {
      return notification;
   }

   public void addCallback(Runnable callback)
   {
      callbacks.add(callback);
   }

   public synchronized void set()
   {
      notification.set();

      for (Runnable callback : callbacks)
      {
         callback.run();
      }
   }
}
