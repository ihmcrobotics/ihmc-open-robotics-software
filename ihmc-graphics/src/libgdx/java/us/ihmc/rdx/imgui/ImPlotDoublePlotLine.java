package us.ihmc.rdx.imgui;

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
}
