package us.ihmc.rdx.ui;

import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.tools.Timer;

import javax.annotation.Nullable;

public class RDX3DPanelNotification extends RDX3DPanelTooltip
{
   private static final double NOTIFICATION_DURATION = 3.0;

   private final Timer timer = new Timer();
   @Nullable
   private String text;

   public RDX3DPanelNotification(RDX3DPanel panel3D)
   {
      super(panel3D);
      super.setInput(new ImGui3DViewInput(panel3D));
   }

   public void render()
   {
      if (timer.getElapsedTime() < NOTIFICATION_DURATION && text != null)
      {
         super.render(text);
      }
   }

   public void setText(@Nullable String text)
   {
      this.text = text;
      reset();
   }

   @Nullable
   public String getText()
   {
      return text;
   }

   public void reset()
   {
      timer.reset();
   }
}
