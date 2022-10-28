package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;

public class ImGuiEnumPlot
{
   private final String label;
   private final int bufferSize;
   private final float[] values;
   private final int width;
   private final int height;
   private int index = 0;

   public ImGuiEnumPlot()
   {
      this(1000, 250, 50);
   }

   public ImGuiEnumPlot(int bufferSize, int width, int height)
   {
      this("", bufferSize, width, height);
   }

   public ImGuiEnumPlot(String label, int bufferSize, int width, int height)
   {
      this.label = label;
      this.bufferSize = bufferSize;
      values = new float[bufferSize];
      this.width = width;
      this.height = height;
      Arrays.fill(values, Float.NaN);
   }

   public void render(int ordinal, String overlayText)
   {
      if (ordinal < 0)
         values[index] = Float.NaN;
      else
         values[index] = ordinal;

      ImGui.plotLines(label, values, bufferSize, 0, overlayText, Float.MAX_VALUE, Float.MAX_VALUE, width, height);

      ++index;
      if (index >= bufferSize - 1)
      {
         index = 0;
      }
      values[index] = Float.NaN;
   }
}
