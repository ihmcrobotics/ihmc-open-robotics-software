package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImGuiAveragedFrequencyText
{
   public static final int HISTORY = 100;
   private final FrequencyCalculator averagedFrequencyCalculator = new FrequencyCalculator(HISTORY);
   private final Throttler throttler = new Throttler().setFrequency(1.0);
   private String fpsString = "0 Hz";

   public void ping()
   {
      averagedFrequencyCalculator.ping();
   }

   public void render()
   {
      if (throttler.run())
      {
         fpsString = String.valueOf((int) averagedFrequencyCalculator.getFrequency());
         while (fpsString.length() < 3)
         {
            fpsString = " " + fpsString;
         }
         fpsString += " Hz";
      }
      ImGui.text(fpsString);
   }
}
