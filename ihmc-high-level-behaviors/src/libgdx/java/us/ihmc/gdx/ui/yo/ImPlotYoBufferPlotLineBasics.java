package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;

public abstract class ImPlotYoBufferPlotLineBasics implements ImPlotPlotLine
{
   private Runnable legendPopupImGuiRenderer;
   private final String variableNameBase;
   private final String variableNamePostfix;
   private final String variableName;
   private final String labelID;

   public ImPlotYoBufferPlotLineBasics(String variableName, String initialValueString)
   {
      this.variableName = variableName;
      variableNameBase = variableName + " ";
      variableNamePostfix = "###" + variableName;
      labelID = variableNameBase + initialValueString + variableNamePostfix;
   }

   protected abstract void plot(String labelID);

   protected abstract void update();

   @Override
   public boolean render()
   {
      update();
      plot(labelID);

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

   @Override
   public String getVariableName()
   {
      return variableName;
   }
}
