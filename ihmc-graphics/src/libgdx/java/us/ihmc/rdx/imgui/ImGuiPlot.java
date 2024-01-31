package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

public class ImGuiPlot
{
   private static final AtomicInteger ID = new AtomicInteger();

   private final String name;
   private final int bufferSize;
   private final float[] values;
   private int width;
   private int height;
   private int index = 0;

   public ImGuiPlot(String name)
   {
      this(name, 1000);
   }

   public ImGuiPlot(String name, int bufferSize)
   {
      this(name, bufferSize, 230, 50);
   }

   public ImGuiPlot(String name, int bufferSize, int width, int height)
   {
      this.name = ImGuiTools.uniqueLabel(getClass().getSimpleName() + ID.getAndIncrement(), name);
      this.bufferSize = bufferSize;
      values = new float[bufferSize];
      this.width = width;
      this.height = height;
      Arrays.fill(values, Float.NaN);
   }

   public void setValue(double newValue)
   {
      setValue((float) newValue);
   }

   public void setValue(float newValue)
   {
      values[index] = newValue;
   }

   public void render(double newValue)
   {
      render((float) newValue);
   }

   public void render(float newValue)
   {
      setValue(newValue);
      render();
   }

   public void render()
   {
      ImGui.plotLines(name, values, bufferSize, 0, "" + values[index], Float.MAX_VALUE, Float.MAX_VALUE, width, height);

      ++index;
      if (index >= bufferSize - 1)
      {
         index = 0;
      }
      values[index] = Float.NaN;
   }

   public void setWidth(int width)
   {
      this.width = width;
   }

   public void setHeight(int height)
   {
      this.height = height;
   }
}
