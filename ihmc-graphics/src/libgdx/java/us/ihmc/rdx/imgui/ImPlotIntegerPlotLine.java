package us.ihmc.rdx.imgui;

public class ImPlotIntegerPlotLine extends ImPlotWallTimeScrollingPlotLine
{
   private final ImPlotPlotLineIntegerSwapBuffer integerSwapBuffer;

   public ImPlotIntegerPlotLine(String variableName)
   {
      this(variableName, 100, 3.0);
   }

   public ImPlotIntegerPlotLine(String variableName, int bufferSize, double history)
   {
      super(variableName, "0", bufferSize, history);

      integerSwapBuffer = new ImPlotPlotLineIntegerSwapBuffer();
      setSwapBuffer(integerSwapBuffer);
   }

   public void addValue(int value)
   {
      integerSwapBuffer.addValue(value);
      super.addValue(String.valueOf(value));
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return String.valueOf(integerSwapBuffer.getValue(bufferIndex));
   }
}
