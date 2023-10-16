package us.ihmc.rdx.ui;

import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.tools.Timer;

import javax.annotation.Nullable;

public class RDX3DPanelNotification extends RDX3DPanelTooltip
{
   public static final double NOTIFICATION_DURATION = 3.0;

   private final Timer timer = new Timer();
   private String text;

   public RDX3DPanelNotification(RDX3DPanel panel3D, String text)
   {
      super(panel3D);
      this.text = text;
      super.setInput(new ImGui3DViewInput(panel3D));
      timer.reset();
   }

   public void render(int notificationIndex)
   {
      super.render(text, notificationIndex);
   }

   public Timer getTimer()
   {
      return timer;
   }

   public void setText(@Nullable String text)
   {
      this.text = text;
   }

   @Nullable
   public String getText()
   {
      return text;
   }
}
