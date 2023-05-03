package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;

public class ImGuiEnumPlot extends ImGuiFancyWidget
{
   private final int bufferSize;
   private final float[] values;
   private final int height;
   private int index = 0;

   public ImGuiEnumPlot()
   {
      this(1000, 0, 20);
   }

   /**
    * Auto-fit width.
    */
   public ImGuiEnumPlot(String label, int bufferSize, int height)
   {
      this(label, bufferSize, 0, height);

   }
   public ImGuiEnumPlot(int bufferSize, int width, int height)
   {
      this("", bufferSize, width, height);
   }

   public ImGuiEnumPlot(String label, int bufferSize, int width, int height)
   {
      super(label);
      this.bufferSize = bufferSize;
      values = new float[bufferSize];
      this.height = height;
      Arrays.fill(values, Float.NaN);
   }

   public void render(int ordinal, String overlayText)
   {
      beforeWidgetRender();

      if (ordinal < 0)
         values[index] = Float.NaN;
      else
         values[index] = ordinal;

      ImGui.plotLines(label, values, bufferSize, 0, overlayText, Float.MAX_VALUE, Float.MAX_VALUE, 0, height);

      ++index;
      if (index >= bufferSize - 1)
      {
         index = 0;
      }
      values[index] = Float.NaN;

      afterWidgetRender();
   }
}
