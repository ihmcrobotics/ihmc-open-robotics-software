package us.ihmc.rdx.ui.yo;

import imgui.extension.implot.ImPlot;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.rdx.imgui.ImPlotPlotLine;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public abstract class ImPlotYoBufferPlotLineBasics implements ImPlotPlotLine
{
   private final YoVariable yoVariable;
   private final String variableNameBase;
   private final String variableNamePostfix;
   private String labelID;
   private final Consumer<YoVariable> removeSelf;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public ImPlotYoBufferPlotLineBasics(YoVariable yoVariable, String initialValueString, Consumer<YoVariable> removeSelf)
   {
      this.yoVariable = yoVariable;
      variableNameBase = yoVariable.getName() + " ";
      variableNamePostfix = "###" + yoVariable.getFullNameString();
      labelID = variableNameBase + initialValueString + variableNamePostfix;
      this.removeSelf = removeSelf;
   }

   public abstract void setupLinkedVariable(RDXYoManager yoManager);

   protected abstract void plot(String labelID);

   protected abstract void update();

   @Override
   public boolean render()
   {
      update();
      labelID = variableNameBase + yoVariable.getValueAsString("%.5f") + variableNamePostfix;
      plot(labelID);

      boolean showingLegendPopup = false;
      if (ImPlot.beginLegendPopup(labelID))
      {
         showingLegendPopup = true;
         ImGui.text(yoVariable.getFullNameString());
         if (yoVariable.getDescription() != null && !yoVariable.getDescription().isEmpty())
         {
            ImGui.textWrapped(yoVariable.getDescription());
         }
         ImGui.separator();
         if (ImGui.menuItem(labels.get("Remove variable from plot")))
         {
            removeSelf.accept(yoVariable);
         }
         ImPlot.endLegendPopup();
      }
      return showingLegendPopup;
   }

   @Override
   public String getVariableName()
   {
      return yoVariable.getName();
   }
}
