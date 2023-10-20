package us.ihmc.rdx.imgui;

import imgui.extension.implot.ImPlot;

public class ImPlotPlotLineIntegerSwapBuffer implements ImPlotPlotLineSwapBuffer
{
   private double[] yValuesA;
   private double[] yValuesB;
   private int bufferSize;
   private int value = 0;
   private boolean isA = true;

   @Override
   public void initialize(int bufferSize)
   {
      this.bufferSize = bufferSize;
      yValuesA = ImPlotTools.newZeroFilledBuffer(bufferSize);
      yValuesB = ImPlotTools.newZeroFilledBuffer(bufferSize);
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
      double[] previousValues = isA ? yValuesA : yValuesB;
      double[] updatedValues = isA ? yValuesB : yValuesA;
      System.arraycopy(previousValues, srcPos, updatedValues, 0, length);
      isA = !isA;
   }

   @Override
   public void plot(String labelID, double[] xValues, int offset)
   {
      ImPlot.plotLine(labelID, xValues, isA ? yValuesA : yValuesB, xValues.length, offset);
   }

   public int getValue(int bufferIndex)
   {
      return (int) (isA ? yValuesA[bufferIndex] : yValuesB[bufferIndex]);
   }
}
