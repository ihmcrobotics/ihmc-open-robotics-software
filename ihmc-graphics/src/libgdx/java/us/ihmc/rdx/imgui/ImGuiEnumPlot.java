package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;

public class ImGuiEnumPlot extends ImGuiFancyWidget
{
   /** For things happening at > 1 Hz, 1000 is a good out of the hat buffer size. */
   public static final int TYPICAL_BUFFER_SIZE = 1000;
   /** This is just a value approximately equal to the typical font height. */
   public static final int TYPICAL_PLOT_HEIGHT = 20;
   /** Set the width to this to fill available space. */
   public static final int AUTO_SIZE_WIDTH = 0;

   private final int bufferSize;
   private final float[] values;
   private final int height;
   private int index = 0;

   public ImGuiEnumPlot()
   {
      this(TYPICAL_BUFFER_SIZE, AUTO_SIZE_WIDTH, TYPICAL_PLOT_HEIGHT);
   }


   public ImGuiEnumPlot(String label)
   {
      this(label, TYPICAL_BUFFER_SIZE, AUTO_SIZE_WIDTH, TYPICAL_PLOT_HEIGHT);
   }

   /**
    * Auto-fit width.
    */
   public ImGuiEnumPlot(String label, int bufferSize, int height)
   {
      this(label, bufferSize, AUTO_SIZE_WIDTH, height);

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

      ImGui.pushItemWidth(ImGui.getColumnWidth());
      ImGui.plotLines(label, values, bufferSize, 0, overlayText, Float.MAX_VALUE, Float.MAX_VALUE, 0, height);
      ImGui.popItemWidth();

      ++index;
      if (index >= bufferSize - 1)
      {
         index = 0;
      }
      values[index] = Float.NaN;

      afterWidgetRender();
   }
}
