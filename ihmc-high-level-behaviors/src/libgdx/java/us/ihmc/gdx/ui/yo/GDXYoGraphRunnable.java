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

   private final ImPlotContext context;
   private final AtomicInteger currentIndex;
   private final ArrayList<YoVariable> variables = new ArrayList<>();
   private final ArrayList<Double[]> values = new ArrayList<>();
   private final YoRegistry registry;
   private final int bufferSize;

   private final int plotID;
   private static int currentPlotIndex = 0;

   public GDXYoGraphRunnable(ImPlotContext ctx, YoRegistry registry, int bufferSize) {
      this.context = ctx;
      this.registry = registry;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public GDXYoGraphRunnable(ImPlotContext ctx, YoVariable variable, Double[] values, YoRegistry registry, int bufferSize) {
      this.context = ctx;
      this.variables.add(variable);
      this.values.add(values);
      this.registry = registry;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public boolean shouldGraphExist() {
      return shouldGraphExist;
   }

   @Override
   public void run()
   {
      if (!shouldGraphExist)
         return;

      ImPlot.setCurrentContext(context);

      int currentValueIndex = currentIndex.getAndIncrement();
      float graphWidth = ImGui.getWindowSizeX() - 45;
      float graphHeight = 60;
      ImPlot.pushStyleVar(ImPlotStyleVar.LabelPadding, new ImVec2(0, 0));
      ImPlot.pushStyleVar(ImPlotStyleVar.LegendPadding, new ImVec2(5, 0));
      if (ImPlot.beginPlot("##GDXYoGraph" + plotID, "", "", new ImVec2(graphWidth, graphHeight),
                           ImPlotFlags.NoMenus | ImPlotFlags.NoBoxSelect, ImPlotAxisFlags.NoDecorations | ImPlotAxisFlags.AutoFit, ImPlotAxisFlags.NoLabel | ImPlotAxisFlags.NoGridLines | ImPlotAxisFlags.NoTickMarks | ImPlotAxisFlags.NoTickLabels | ImPlotAxisFlags.AutoFit))
      {
         ImPlot.setLegendLocation(ImPlotLocation.SouthWest, ImPlotOrientation.Horizontal, true);

         Iterator<YoVariable> itVar = variables.iterator();
         Iterator<Double[]> itVal = values.iterator();
         while (itVar.hasNext())
         {
            YoVariable variable = itVar.next();
            Double[] varValues = itVal.next();
            varValues[currentValueIndex] = variable.getValueAsDouble();
            Double[] data = ImPlotTools.removeNullElements(varValues);

            ImPlot.plotLine(variable.getName(), ImPlotTools.createIndex(data), data);
            if (ImPlot.beginLegendPopup(variable.getName()))
            {
               ImGui.text(variable.getFullNameString());
               if (ImGui.button("Stop tracking variable##" + variable.getName()))
               {
                  itVar.remove();
                  itVal.remove();
               }
               ImPlot.endLegendPopup();
            }
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

      ImGui.sameLine();
      if (ImGui.button("X##" + plotID, 20, graphHeight))
      {
         shouldGraphExist = false;
      }

      if (currentValueIndex >= bufferSize - 1)
      {
         currentIndex.set(0);
      }
   }
}