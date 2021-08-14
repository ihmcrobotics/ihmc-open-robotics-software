package us.ihmc.gdx.ui.yo;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.extension.implot.flag.*;
import imgui.internal.ImGui;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.atomic.AtomicInteger;

public class GDXYoGraphRunnable implements Runnable
{
   private boolean shouldGraphExist = true;
   private boolean requestAddVariable = false;

   private final ImPlotContext imPlotContext;
   private final AtomicInteger currentIndex;
   private final ArrayList<YoVariable> variables = new ArrayList<>();
   private final ArrayList<Double[]> variableValueBuffers = new ArrayList<>();
   private final YoRegistry registry;
   private final int bufferSize;

   private final int plotID;
   private static int currentPlotIndex = 0;

   public GDXYoGraphRunnable(ImPlotContext imPlotContext, YoRegistry registry, int bufferSize)
   {
      this.imPlotContext = imPlotContext;
      this.registry = registry;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public GDXYoGraphRunnable(ImPlotContext imPlotContext, YoVariable variable, Double[] variableValueBuffer, YoRegistry registry, int bufferSize)
   {
      this.imPlotContext = imPlotContext;
      this.variables.add(variable);
      this.variableValueBuffers.add(variableValueBuffer);
      this.registry = registry;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public boolean shouldGraphExist()
   {
      return shouldGraphExist;
   }

   public void cancelWantVariable()
   {
      requestAddVariable = false;
   }

   public boolean graphWantsVariable()
   {
      return requestAddVariable;
   }

   public void addVariable(YoVariable variable)
   {
      variables.add(variable);
      variableValueBuffers.add(new Double[bufferSize]);
      requestAddVariable = false;
   }

   @Override
   public void run()
   {
      if (!shouldGraphExist)
         return;

      ImPlot.setCurrentContext(imPlotContext);

      int currentValueIndex = currentIndex.getAndIncrement();
      float graphWidth = ImGui.getColumnWidth();
      float graphHeight = 60;
      ImPlot.pushStyleVar(ImPlotStyleVar.LabelPadding, new ImVec2(0, 0));
      ImPlot.pushStyleVar(ImPlotStyleVar.LegendPadding, new ImVec2(5, 0));
      if (ImPlot.beginPlot("##GDXYoGraph" + plotID,
                           "",
                           "",
                           new ImVec2(graphWidth, graphHeight),
                           ImPlotFlags.NoMenus | ImPlotFlags.NoBoxSelect,
                           ImPlotAxisFlags.NoDecorations | ImPlotAxisFlags.AutoFit,
                           ImPlotAxisFlags.NoLabel | ImPlotAxisFlags.NoGridLines | ImPlotAxisFlags.NoTickMarks | ImPlotAxisFlags.NoTickLabels
                           | ImPlotAxisFlags.AutoFit))
      {
         ImPlot.setLegendLocation(ImPlotLocation.SouthWest, ImPlotOrientation.Horizontal, true);

         Iterator<YoVariable> variableIterator = variables.iterator();
         Iterator<Double[]> variableValueBufferIterator = variableValueBuffers.iterator();
         boolean showingLegendPopup = false;
         while (variableIterator.hasNext())
         {
            YoVariable variable = variableIterator.next();
            Double[] variableValueBuffer = variableValueBufferIterator.next();
            variableValueBuffer[currentValueIndex] = variable.getValueAsDouble();
            Double[] variableValueBufferWithoutNulls = ImPlotTools.removeNullElements(variableValueBuffer);

            String labelID = variable.getName() + " " + variable.getValueAsDouble() + "###" + variable.getName();
            ImPlot.plotLine(labelID, ImPlotTools.createIndex(variableValueBufferWithoutNulls), variableValueBufferWithoutNulls);
            if (ImPlot.beginLegendPopup(labelID))
            {
               showingLegendPopup = true;
               ImGui.text(variable.getFullNameString());
               if (variable.getDescription() != null && !variable.getDescription().isEmpty())
               {
                  ImGui.separator();
                  ImGui.textWrapped(variable.getDescription());
               }
               if (ImGui.button("Stop tracking variable##" + variable.getName()))
               {
                  variableIterator.remove();
                  variableValueBufferIterator.remove();
               }
               ImPlot.endLegendPopup();
            }
         }

         if (!showingLegendPopup && ImGui.beginPopupContextWindow())
         {
            if (ImGui.button("Add a variable...##" + plotID))
            {
               requestAddVariable = true;
            }

            if (ImGui.button("Remove graph##" + plotID))
            {
               shouldGraphExist = false;
            }

            ImGui.endPopup();
         }

         if (ImPlot.beginDragDropTarget())
         {
            String payload = ImGui.acceptDragDropPayload(String.class);
            if (payload != null && registry.hasVariable(payload) && !variables.contains(registry.findVariable(payload)))
            {
               variables.add(registry.findVariable(payload));
               variableValueBuffers.add(new Double[bufferSize]);
            }
         }
         ImPlot.endPlot();
      }
      ImPlot.popStyleVar(2);

      if (currentValueIndex >= bufferSize - 1)
      {
         currentIndex.set(0);
      }
   }
}