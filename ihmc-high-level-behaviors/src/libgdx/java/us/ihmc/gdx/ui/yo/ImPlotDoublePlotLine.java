package us.ihmc.gdx.ui.yo;

import java.text.DecimalFormat;

public class ImPlotDoublePlotLine extends ImPlotPlotLineBasics
{
   private static final DecimalFormat decimal5DPrintFormatter = new DecimalFormat("0.00000");
   private final ImPlotPlotLineDoubleSwapBuffer doubleSwapBuffer;

   public ImPlotDoublePlotLine(String variableName)
   {
      this(variableName, 100, 3.0);
   }

   public ImPlotDoublePlotLine(String variableName, int bufferSize, double history)
   {
      super(variableName, "NaN", bufferSize, history);

      doubleSwapBuffer = new ImPlotPlotLineDoubleSwapBuffer();
      setSwapBuffer(doubleSwapBuffer);
   }

   public void addValue(double value)
   {
      doubleSwapBuffer.addValue(value);
      super.addValue(decimal5DPrintFormatter.format(value));
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return decimal5DPrintFormatter.format(doubleSwapBuffer.getValue(bufferIndex));
   }
}
