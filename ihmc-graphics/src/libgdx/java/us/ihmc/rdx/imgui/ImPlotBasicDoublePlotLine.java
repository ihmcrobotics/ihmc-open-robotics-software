package us.ihmc.rdx.imgui;

public class ImPlotBasicDoublePlotLine implements ImPlotPlotLine
{
   private double[] yValues;
   private int bufferSize;

   public void addValue(double newValue)
   {
//      System.arraycopy(previousValues, srcPos, updatedValues, 0, length);

   }

   @Override
   public boolean render()
   {
//      ImPlot.plotLine(labelID, xValues, yValues.getForThreadOne(), xValues.length, offset);
      return false;
   }

   @Override
   public String getVariableName()
   {
      return "";
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return null;
   }
}
