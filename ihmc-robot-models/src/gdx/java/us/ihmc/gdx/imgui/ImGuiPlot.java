package us.ihmc.gdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;

public class ImGuiPlot
{
   private final String name;
   private final int bufferSize;
   private final float[] values;
   private int index = 0;

   public ImGuiPlot(String name, int bufferSize)
   {
      this.name = name;
      this.bufferSize = bufferSize;
      values = new float[bufferSize];
      Arrays.fill(values, Float.NaN);
   }

   public void setValue(float newValue)
   {
      values[index] = newValue;
   }

   public void render(float newValue)
   {
      setValue(newValue);
      render();
   }

   public void render()
   {
      int graphWidth = 230;
      int graphHeight = 50;
      ImGui.plotLines(name, values, bufferSize, 0, "" + values[index], Float.MAX_VALUE, Float.MAX_VALUE, graphWidth, graphHeight);

      ++index;
      if (index >= bufferSize - 1)
      {
         index = 0;
      }
      values[index] = Float.NaN;
   }
}
