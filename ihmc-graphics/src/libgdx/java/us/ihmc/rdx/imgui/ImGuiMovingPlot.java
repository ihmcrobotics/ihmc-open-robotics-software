package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

public class ImGuiMovingPlot
{
   private static final AtomicInteger ID = new AtomicInteger();

   private final String name;
   private final int bufferSize;
   private final float[] valuesA;
   private final float[] valuesB;
   private boolean isA = true;
   private final int width;
   private final int height;
   private float nextValue = Float.NaN;
   private boolean renderValueText = true;

   public ImGuiMovingPlot(String name)
   {
      this(name, 1000);
   }

   public ImGuiMovingPlot(String name, int bufferSize)
   {
      this(name, bufferSize, 230, 50);
   }

   public ImGuiMovingPlot(String name, int bufferSize, int width, int height)
   {
      this.name = ImGuiTools.uniqueLabel(getClass().getSimpleName() + ID.getAndIncrement(), name);
      this.bufferSize = bufferSize;
      valuesA = new float[bufferSize];
      valuesB = new float[bufferSize];
      this.width = width;
      this.height = height;
      Arrays.fill(valuesA, Float.NaN);
      Arrays.fill(valuesB, Float.NaN);
   }

   public void setRenderValueText(boolean renderValueText)
   {
      this.renderValueText = renderValueText;
   }

   public void setNextValue(float newValue)
   {
      nextValue = newValue;
   }

   public void calculate(float newValue)
   {
      setNextValue(newValue);
      calculate();
   }

   public void calculate()
   {
      String valueText = "";
      if (renderValueText)
         valueText += nextValue;
      calculate(valueText, true);
   }

   public void calculate(String valueText) {
      calculate(valueText, true);
   }

   public void calculate(String valueText, boolean render)
   {
      System.arraycopy(isA ? valuesB : valuesA, 1, isA ? valuesA : valuesB, 0, bufferSize - 1);
      float[] values = isA ? valuesA : valuesB;
      isA = !isA;
      values[bufferSize - 1] = nextValue;
      if (render)
         ImGui.plotLines(name, values, bufferSize, 0, valueText, Float.MAX_VALUE, Float.MAX_VALUE, width, height);

      setNextValue(Float.NaN);
   }
}
