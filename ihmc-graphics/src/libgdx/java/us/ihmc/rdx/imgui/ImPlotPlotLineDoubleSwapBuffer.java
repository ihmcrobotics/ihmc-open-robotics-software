package us.ihmc.rdx.imgui;

import imgui.extension.implot.ImPlot;
import us.ihmc.tools.thread.SwapReference;

public class ImPlotPlotLineDoubleSwapBuffer implements ImPlotPlotLineSwapBuffer
{
   private SwapReference<double[]> yValues;
   private int bufferSize;
   private double value = Double.NaN;

   @Override
   public void initialize(int bufferSize)
   {
      this.bufferSize = bufferSize;
      yValues = new SwapReference<>(() -> ImPlotTools.newNaNFilledBuffer(bufferSize));
   }

   public void addValue(double value)
   {
      this.value = value;
   }

   @Override
   public void setAValue(int index)
   {
      yValues.getA()[index] = value;
   }

   @Override
   public void setPreviousValue(int index)
   {
      yValues.getForThreadOne()[index] = value;
   }

   @Override
   public void setUpdatedValue(int index)
   {
      yValues.getForThreadTwo()[index] = value;
   }

   @Override
   public void copyAToB()
   {
      System.arraycopy(yValues.getA(), 0, yValues.getB(), 0, bufferSize);
   }

   @Override
   public void copyPreviousToUpdated(int srcPos, int destPos, int length)
   {
      double[] previousValues = yValues.getForThreadOne();
      double[] updatedValues = yValues.getForThreadTwo();
      System.arraycopy(previousValues, srcPos, updatedValues, 0, length);
      yValues.swap();
   }

   @Override
   public void plot(String labelID, double[] xValues, int offset)
   {
      ImPlot.plotLine(labelID, xValues, yValues.getForThreadOne(), xValues.length, offset);
   }

   public double getValue(int bufferIndex)
   {
      return yValues.getForThreadOne()[bufferIndex];
   }
}
