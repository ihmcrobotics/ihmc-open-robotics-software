package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImGuiAveragedFrequencyText
{
   public static final int HISTORY = 20;
   private final Throttler throttler = new Throttler().setFrequency(0.1);
   private FrequencyCalculator averagedFrequencyCalculator = new FrequencyCalculator(HISTORY);
   private String fpsString = "0 Hz";

   public void ping()
   {
      averagedFrequencyCalculator.ping();
   }

   public void render()
   {
      if (throttler.run())
      {
         fpsString = "%3d Hz".formatted((int) averagedFrequencyCalculator.getFrequency());
      }
      ImGui.text(fpsString.trim());
   }

   public void reset()
   {
      averagedFrequencyCalculator = new FrequencyCalculator(HISTORY);
   }
}
