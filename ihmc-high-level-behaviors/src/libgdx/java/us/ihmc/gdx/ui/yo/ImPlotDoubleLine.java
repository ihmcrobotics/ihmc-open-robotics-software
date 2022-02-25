package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.ui.tools.ImPlotTools;

import java.text.DecimalFormat;

public class ImPlotDoubleLine
{
   private Runnable legendPopupImGuiRenderer;
   private String variableNameBase;
   private String variableNamePostfix;
   private final String variableName;
   private String labelID;
   private static final DecimalFormat decimal5DPrintFormatter = new DecimalFormat("0.00000");
   private int bufferSize = 1000;
   private final Integer[] xValues = ImPlotTools.createIndex(bufferSize);
   private final Double[] yValuesA = ImPlotTools.newNaNFilledBuffer(bufferSize);
   private final Double[] yValuesB = ImPlotTools.newNaNFilledBuffer(bufferSize);
   private boolean isA = true;
   private int filledIndex = 0;

   public ImPlotDoubleLine(String variableName)
   {
      this.variableName = variableName;
      variableNameBase = variableName + " ";
      variableNamePostfix = "###" + variableName;
      labelID = variableNameBase + "NaN" + variableNamePostfix;
   }

   public void addValue(double value)
   {
      labelID = variableNameBase + decimal5DPrintFormatter.format(value) + variableNamePostfix;

      if (filledIndex < bufferSize)
      {
         yValuesA[filledIndex] = value;
         ++filledIndex;

         if (filledIndex == bufferSize)
         {
            System.arraycopy(yValuesA, 0, yValuesB, 0, bufferSize);
         }
      }
      else
      {
         Double[] previousValues = isA ? yValuesB : yValuesA;
         Double[] updatedValues = isA ? yValuesA : yValuesB;
         System.arraycopy(previousValues, 1, updatedValues, 0, bufferSize - 1);
         isA = !isA;
         updatedValues[bufferSize - 1] = value;
      }
   }

   public boolean render()
   {
      int offset = 0; // This is believed to be the index in the array we are passing in which implot will start reading
      ImPlot.plotLine(labelID, xValues, isA ? yValuesA : yValuesB, offset);

      boolean showingLegendPopup = false;
      if (legendPopupImGuiRenderer != null && ImPlot.beginLegendPopup(labelID))
      {
         showingLegendPopup = true;
         legendPopupImGuiRenderer.run();
         ImPlot.endLegendPopup();
      }
      return showingLegendPopup;
   }

   public void setLegendPopupImGuiRenderer(Runnable legendPopupImGuiRenderer)
   {
      this.legendPopupImGuiRenderer = legendPopupImGuiRenderer;
   }
}
