package us.ihmc.rdx.ui;

import java.util.LinkedList;
import java.util.List;

public class RDX3DPanelNotificationManager
{
   private final List<RDX3DPanelNotification> removeQueue = new LinkedList<>();
   private final List<RDX3DPanelNotification> addQueue = new LinkedList<>();
   private final List<RDX3DPanelNotification> notificationQueue = new LinkedList<>();

   private RDX3DPanel panel3D;

   public RDX3DPanelNotificationManager(RDX3DPanel panel3)
   {
      this.panel3D = panel3;
   }

   public void pushNotification(String text)
   {
      addQueue.add(new RDX3DPanelNotification(panel3D, text));
   }

   public void render()
   {
      // Handle removes
      for (RDX3DPanelNotification remove : removeQueue)
         notificationQueue.remove(remove);
      removeQueue.clear();

      // Handle adds
      notificationQueue.addAll(addQueue);
      addQueue.clear();

      for (int i = 0; i < notificationQueue.size(); i++)
      {
         RDX3DPanelNotification notification = notificationQueue.get(i);

         if (notification.getTimer().getElapsedTime() > RDX3DPanelNotification.NOTIFICATION_DURATION)
         {
            // Add to remove queue
            removeQueue.add(notification);
         }
         else
         {
            notification.render(i);
         }
      }
   }
}
