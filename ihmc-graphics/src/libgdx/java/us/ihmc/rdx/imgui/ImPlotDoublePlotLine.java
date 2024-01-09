package us.ihmc.rdx.imgui;

import imgui.extension.implot.ImPlot;
import imgui.flag.ImGuiCond;

import java.text.DecimalFormat;

public class ImPlotDoublePlotLine extends ImPlotWallTimeScrollingPlotLine
{
   private final DecimalFormat decimalFormatter;
   private final ImPlotPlotLineDoubleSwapBuffer doubleSwapBuffer;

   public ImPlotDoublePlotLine(String variableName)
   {
      this(variableName, 100, 3.0, new DecimalFormat("0.00000"));
   }

   public ImPlotDoublePlotLine(String variableName, int bufferSize, double history, DecimalFormat decimalFormatter)
   {
      super(variableName, "NaN", bufferSize, history);
      this.decimalFormatter = decimalFormatter;

      doubleSwapBuffer = new ImPlotPlotLineDoubleSwapBuffer();
      setSwapBuffer(doubleSwapBuffer);
   }

   public void addValue(double value)
   {
      doubleSwapBuffer.addValue(value);
      super.addValue(decimalFormatter.format(value));
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return decimalFormatter.format(doubleSwapBuffer.getValue(bufferIndex));
   }

   public void setLimitYMin(double minLimitY)
   {
      double limitY = minLimitY;
      for (int i = 0; i < getBufferSize(); i++)
         limitY = Math.max(doubleSwapBuffer.getValue(i), limitY);
      ImPlot.setNextPlotLimitsY(0.0, limitY, ImGuiCond.Always);
   }
}
