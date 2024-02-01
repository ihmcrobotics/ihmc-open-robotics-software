package us.ihmc.rdx.ui.yo;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.extension.implot.flag.*;
import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.rdx.imgui.ImPlotTools;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.atomic.AtomicInteger;

public class ImPlotYoGraph
{
   private boolean shouldGraphExist = true;
   private boolean requestAddVariable = false;

   private final AtomicInteger currentIndex;
   private final ArrayList<YoDoubleClientHelper> variables = new ArrayList<>();
   private final ArrayList<double[]> variableValueBuffers = new ArrayList<>();
   private final YoVariableClientHelper helper;
   private final int bufferSize;

   private final int plotID;
   private static int currentPlotIndex = 0;

   public ImPlotYoGraph(YoVariableClientHelper helper, int bufferSize)
   {
      this.helper = helper;
      this.bufferSize = bufferSize;

      plotID = currentPlotIndex++;
      currentIndex = new AtomicInteger(0);
   }

   public ImPlotYoGraph(YoDoubleClientHelper variable, YoVariableClientHelper helper, double[] variableValueBuffer, int bufferSize)
   {
      this.helper = helper;
      this.variables.add(variable);
      this.variableValueBuffers.add(variableValueBuffer);
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

   public void addVariable(String variableName)
   {
      YoDoubleClientHelper variable = helper.subscribeToYoDouble(variableName);
      variables.add(variable);
      variableValueBuffers.add(ImPlotTools.newNaNFilledBuffer(bufferSize));
      requestAddVariable = false;
   }

   public void render(ImPlotContext imPlotContext)
   {
      if (!shouldGraphExist)
         return;

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

         Iterator<YoDoubleClientHelper> variableIterator = variables.iterator();
         Iterator<double[]> variableValueBufferIterator = variableValueBuffers.iterator();
         boolean showingLegendPopup = false;
         while (variableIterator.hasNext())
         {
            YoDoubleClientHelper variable = variableIterator.next();
            double[] variableValueBuffer = variableValueBufferIterator.next();
            double variableValue = variable.get();
            variableValueBuffer[currentValueIndex] = variableValue;

            String labelID = variable.getName() + " " + variableValue + "###" + variable.getName();
            ImPlot.plotLine(labelID, ImPlotTools.createIndex(variableValueBuffer.length), variableValueBuffer, variableValueBuffer.length, 0);
            if (ImPlot.beginLegendPopup(labelID))
            {
               showingLegendPopup = true;
               ImGui.text(variable.getFullName());
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

            if (payload != null)
            {
               YoDoubleClientHelper yoDoubleHelper = helper.subscribeToYoDouble(payload);
               variables.add(yoDoubleHelper);
               variableValueBuffers.add(ImPlotTools.newNaNFilledBuffer(bufferSize));
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