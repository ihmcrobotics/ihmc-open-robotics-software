package us.ihmc.gdx.ui.yo;

public class ImPlotIntegerPlotLine extends ImPlotPlotLineBasics
{
   private final ImPlotPlotLineIntegerSwapBuffer integerSwapBuffer;

   public ImPlotIntegerPlotLine(String variableName)
   {
      super(variableName, "0");

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
