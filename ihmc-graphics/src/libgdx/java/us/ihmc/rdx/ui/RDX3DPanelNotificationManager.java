package us.ihmc.rdx.ui;

import java.util.Deque;
import java.util.LinkedList;

public class RDX3DPanelNotificationManager
{
   private final Deque<RDX3DPanelNotification> notificationDeque = new LinkedList<>();
   private final RDX3DPanel panel3D;

   public RDX3DPanelNotificationManager(RDX3DPanel panel3)
   {
      this.panel3D = panel3;
   }

   public void pushNotification(String text)
   {
      notificationDeque.addLast(new RDX3DPanelNotification(panel3D, text));
   }

   public void render()
   {
      // Handle expiry
      if (!notificationDeque.isEmpty())
         if (notificationDeque.getFirst().getTimer().getElapsedTime() > RDX3DPanelNotification.NOTIFICATION_DURATION)
            notificationDeque.removeFirst();

      // Render remaining notifications
      int index = 0;
      for (RDX3DPanelNotification notification : notificationDeque)
         notification.render(index++);
   }
}
