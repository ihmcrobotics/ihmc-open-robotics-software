package us.ihmc.rdx.imgui;

import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotCol;
import imgui.flag.ImGuiCond;

public class ImPlotBasicDoublePlotLine implements ImPlotPlotLine
{
   private int size = 0;
   private double[] xValues = new double[100];
   private double[] yValues = new double[100];
   private String legendLabel = "";
   private int dataColor = -1;

   public ImPlotBasicDoublePlotLine()
   {
      initializeXValues();
   }

   public void setLegendLabel(String legendLabel)
   {
      this.legendLabel = legendLabel;
   }

   public void setDataColor(int dataColor)
   {
      this.dataColor = dataColor;
   }

   public void clear()
   {
      size = 0;
   }

   private void initializeXValues()
   {
      for (int i = 0; i < xValues.length; i++)
      {
         xValues[i] = i;
      }
   }

   public void addValue(double newValue)
   {
      if (size == xValues.length)
      {
         double[] enlargedXValues = new double[xValues.length * 2];
         System.arraycopy(xValues, 0, enlargedXValues, 0, xValues.length);
         xValues = enlargedXValues;

         double[] enlargedYValues = new double[yValues.length * 2];
         System.arraycopy(yValues, 0, enlargedYValues, 0, yValues.length);
         yValues = enlargedYValues;

         initializeXValues();
      }

      yValues[size] = newValue;
      ++size;
   }

   @Override
   public boolean render()
   {
      if (dataColor != -1)
         ImPlot.pushStyleColor(ImPlotCol.Line, dataColor);

      ImPlot.plotLine(legendLabel, xValues, yValues, size, 0);

      if (dataColor != -1)
         ImPlot.popStyleColor();

      return false;
   }

   @Override
   public String getVariableName()
   {
      return legendLabel;
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return "%.2f".formatted(yValues[bufferIndex]);
   }

   public void setLimitYMin(double minLimitY)
   {
      double plotMaxY = getMaxYValue();
      ImPlot.setNextPlotLimitsY(0.0, Double.isNaN(plotMaxY) ? minLimitY : plotMaxY, ImGuiCond.Always);
   }

   public double getMaxYValue()
   {
      double max = Double.NaN;
      for (int i = 0; i < size; i++)
      {
         if (!Double.isNaN(yValues[i]))
         {
            max = Double.isNaN(max) ? yValues[i] : Math.max(yValues[i], max);
         }
      }
      return max;
   }
}