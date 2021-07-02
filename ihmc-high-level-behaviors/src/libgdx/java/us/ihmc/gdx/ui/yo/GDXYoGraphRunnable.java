package us.ihmc.gdx.ui.yo;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.extension.implot.flag.*;
import imgui.flag.ImGuiDragDropFlags;
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

   private final ImPlotContext context;
   private final AtomicInteger currentIndex;
   private final ArrayList<YoVariable> variables = new ArrayList<>();
   private final ArrayList<Double[]> values = new ArrayList<>();
   private final YoRegistry registry;
   private final int bufferSize;

   private final int plotID;
   private static int currentPlotIndex = 0;

   public GDXYoGraphRunnable(ImPlotContext ctx, YoRegistry registry, int bufferSize)
   {
      this.context = ctx;
      this.registry = registry;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public GDXYoGraphRunnable(ImPlotContext ctx, YoVariable variable, Double[] values, YoRegistry registry, int bufferSize)
   {
      this.context = ctx;
      this.variables.add(variable);
      this.values.add(values);
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
      values.add(new Double[bufferSize]);
      requestAddVariable = false;
   }

   @Override
   public void run()
   {
      if (!shouldGraphExist)
         return;

      ImPlot.setCurrentContext(context);

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

         Iterator<YoVariable> itVar = variables.iterator();
         Iterator<Double[]> itVal = values.iterator();
         boolean showingLegendPopup = false;
         while (itVar.hasNext())
         {
            YoVariable variable = itVar.next();
            Double[] varValues = itVal.next();
            varValues[currentValueIndex] = variable.getValueAsDouble();
            Double[] data = ImPlotTools.removeNullElements(varValues);

            ImPlot.plotLine(variable.getName(), ImPlotTools.createIndex(data), data);
            if (ImPlot.beginLegendPopup(variable.getName()))
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
                  itVar.remove();
                  itVal.remove();
               }
               ImPlot.endLegendPopup();
            }
         }

         if (!showingLegendPopup && ImGui.beginPopupContextWindow()) {
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
               values.add(new Double[bufferSize]);
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