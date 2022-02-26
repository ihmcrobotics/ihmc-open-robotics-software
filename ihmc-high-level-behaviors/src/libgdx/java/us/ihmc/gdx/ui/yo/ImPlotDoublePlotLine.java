package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.ui.tools.ImPlotTools;

import java.text.DecimalFormat;

public class ImPlotDoublePlotLine implements ImPlotPlotLine
{
   private Runnable legendPopupImGuiRenderer;
   private String variableNameBase;
   private String variableNamePostfix;
   private final String variableName;
   private String labelID;
   private static final DecimalFormat decimal5DPrintFormatter = new DecimalFormat("0.00000");
   private double history = 3.0;
   private Stopwatch stopwatch = new Stopwatch();
   private int bufferSize = 100;
   private double timeForOneBufferEntry = history / bufferSize;
   private long lastTickIndex = 0;
   private long tickIndex = 0;
   private final Integer[] xValues = ImPlotTools.createIndex(bufferSize);
   private final Double[] yValuesA = ImPlotTools.newNaNFilledBuffer(bufferSize);
   private final Double[] yValuesB = ImPlotTools.newNaNFilledBuffer(bufferSize);
   private boolean isA = true;
   private int filledIndex = 0;

   public ImPlotDoublePlotLine(String variableName)
   {
      this.variableName = variableName;
      variableNameBase = variableName + " ";
      variableNamePostfix = "###" + variableName;
      labelID = variableNameBase + "NaN" + variableNamePostfix;
   }

   public void addValue(double value)
   {
      labelID = variableNameBase + decimal5DPrintFormatter.format(value) + variableNamePostfix;

      int numberOfTicksToAdvance = 0;

      double totalElapsed = stopwatch.totalElapsed();
      if (Double.isNaN(totalElapsed))
      {
         stopwatch.start();
      }
      else
      {
         tickIndex = (long) (totalElapsed / timeForOneBufferEntry);
         numberOfTicksToAdvance = (int) (tickIndex - lastTickIndex);
         lastTickIndex = tickIndex;
      }

      int numberOfTicksToAdvanceBeforeFilling = Math.min(bufferSize, filledIndex + numberOfTicksToAdvance) - filledIndex;
      int numberOfTicksToAdvanceAfterFilling = numberOfTicksToAdvance - numberOfTicksToAdvanceBeforeFilling;
      if (filledIndex < bufferSize)
      {
         if (numberOfTicksToAdvanceBeforeFilling == 0)
         {
            yValuesA[filledIndex] = value;
         }
         else
         {
            for (int i = 0; i < numberOfTicksToAdvanceBeforeFilling && filledIndex < bufferSize; i++)
            {
               ++filledIndex;
               if (filledIndex < bufferSize)
                  yValuesA[filledIndex] = value;
            }

            if (filledIndex == bufferSize)
            {
               System.arraycopy(yValuesA, 0, yValuesB, 0, bufferSize); // copy A to B when we fill up the first time
            }
         }
      }

      if (filledIndex == bufferSize)
      {
         Double[] previousValues = isA ? yValuesA : yValuesB;
         Double[] updatedValues = isA ? yValuesB : yValuesA;
         if (numberOfTicksToAdvanceAfterFilling > 0)
         {
            System.arraycopy(previousValues, numberOfTicksToAdvanceAfterFilling, updatedValues, 0, bufferSize - numberOfTicksToAdvanceAfterFilling);
            isA = !isA;

            for (int i = 0; i < numberOfTicksToAdvanceAfterFilling; i++)
            {
               updatedValues[bufferSize - 1 - i] = value;
            }
         }
         else
         {
            previousValues[bufferSize - 1] = value;
         }
      }
   }

   @Override
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
