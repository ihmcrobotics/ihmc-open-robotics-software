package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImGuiAveragedFrequencyText
{
   private final FrequencyCalculator averagedFrequencyCalculator = new FrequencyCalculator();
   private long lastQueryTimeMs;
   private String text;

   public void ping()
   {
      averagedFrequencyCalculator.ping();
   }

   public void render()
   {
      ImGui.text(getText());
   }

   public String getText()
   {
      long now = System.currentTimeMillis();

      if (now - lastQueryTimeMs > 500)
      {
         text = "%3d Hz".formatted((int) averagedFrequencyCalculator.getFrequencyDecaying());
         lastQueryTimeMs = now;
      }

      return text;
   }
}
