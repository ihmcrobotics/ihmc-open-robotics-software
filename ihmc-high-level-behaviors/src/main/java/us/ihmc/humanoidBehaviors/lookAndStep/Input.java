package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.commons.thread.Notification;

import java.util.ArrayList;

public class Input
{
   private Notification notification = new Notification();
   private ArrayList<Runnable> callbacks = new ArrayList<>();

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
