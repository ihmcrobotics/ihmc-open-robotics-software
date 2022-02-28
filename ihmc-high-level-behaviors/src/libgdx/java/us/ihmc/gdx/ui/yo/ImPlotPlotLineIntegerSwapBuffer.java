package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.ui.tools.ImPlotTools;

public class ImPlotPlotLineIntegerSwapBuffer implements ImPlotPlotLineSwapBuffer
{
   private Integer[] yValuesA;
   private Integer[] yValuesB;
   private int bufferSize;
   private int value = 0;
   private boolean isA = true;

   @Override
   public void initialize(int bufferSize)
   {
      this.bufferSize = bufferSize;
      yValuesA = ImPlotTools.newZeroFilledIntegerBuffer(bufferSize);
      yValuesB = ImPlotTools.newZeroFilledIntegerBuffer(bufferSize);
   }

   public void addValue(int value)
   {
      this.value = value;
   }

   @Override
   public void setAValue(int index)
   {
      yValuesA[index] = value;
   }

   @Override
   public void setPreviousValue(int index)
   {
      if (isA)
      {
         yValuesA[index] = value;
      }
      else
      {
         yValuesB[index] = value;
      }
   }

   @Override
   public void setUpdatedValue(int index)
   {
      if (isA)
      {
         yValuesB[index] = value;
      }
      else
      {
         yValuesA[index] = value;
      }
   }

   @Override
   public void copyAToB()
   {
      System.arraycopy(yValuesA, 0, yValuesB, 0, bufferSize);
   }

   @Override
   public void copyPreviousToUpdated(int srcPos, int destPos, int length)
   {
      Integer[] previousValues = isA ? yValuesA : yValuesB;
      Integer[] updatedValues = isA ? yValuesB : yValuesA;
      System.arraycopy(previousValues, srcPos, updatedValues, 0, length);
      isA = !isA;
   }

   @Override
   public void plot(String labelID, Integer[] xValues, int offset)
   {
      ImPlot.plotLine(labelID, xValues, isA ? yValuesA : yValuesB, offset);
   }

   public int getValue(int bufferIndex)
   {
      return isA ? yValuesA[bufferIndex] : yValuesB[bufferIndex];
   }
}
