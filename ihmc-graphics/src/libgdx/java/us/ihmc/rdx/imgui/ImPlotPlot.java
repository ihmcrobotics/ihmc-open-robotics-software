package us.ihmc.rdx.imgui;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotPoint;
import imgui.extension.implot.flag.*;
import imgui.ImGui;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

/**
 * Provides a helper layer over {@link ImPlot} which helps use it
 * in the context of common things we've needed so far.
 */
public class ImPlotPlot
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Consumer<String> dragAndDropPayloadConsumer;
   private Runnable popupContextWindowImGuiRenderer;
   private final ArrayList<ImPlotPlotLine> plotLines = new ArrayList<>();
   private final String xLabel;
   private final String yLabel;
   private float graphHeight;
   private float graphWidth;
   private int flags;
   private int xFlags;
   private int yFlags;
   private final ImVec2 outerBoundsDimensionsPixels;
   private final ConcurrentLinkedQueue<ImPlotPlotLine> removalQueue = new ConcurrentLinkedQueue<>();
   private Runnable customBeforePlotLogic = () -> {};
   private Runnable customDuringPlotLogic = () -> {};

   public ImPlotPlot()
   {
      this(60);
   }

   public ImPlotPlot(float plotHeight)
   {
      ImPlotTools.ensureImPlotInitialized();
      ImPlotTools.setSCSStyle();

      xLabel = "";
      yLabel = "";

      flags = ImPlotFlags.None;
      flags += ImPlotFlags.NoMenus;
      flags += ImPlotFlags.NoBoxSelect;
      flags += ImPlotFlags.NoTitle;
      flags += ImPlotFlags.NoMousePos;

      xFlags = ImPlotAxisFlags.None;
      xFlags += ImPlotAxisFlags.NoDecorations;
      xFlags += ImPlotAxisFlags.AutoFit;

      yFlags = ImPlotAxisFlags.None;
      yFlags += ImPlotAxisFlags.NoLabel;
      yFlags += ImPlotAxisFlags.AutoFit;

      graphHeight = plotHeight;
      graphWidth = 100;

      outerBoundsDimensionsPixels = new ImVec2(graphWidth, graphHeight);
   }

   public void render()
   {
      graphWidth = ImGui.getColumnWidth();
      render(graphWidth, graphHeight);
   }

   public void render(float plotWidth, float plotHeight)
   {
      while (!removalQueue.isEmpty())
         plotLines.remove(removalQueue.poll());

      outerBoundsDimensionsPixels.x = plotWidth;
      outerBoundsDimensionsPixels.y = plotHeight;
      customBeforePlotLogic.run();
      if (ImPlot.beginPlot(labels.get("Plot"), xLabel, yLabel, outerBoundsDimensionsPixels, flags, xFlags, yFlags))
      {
         customDuringPlotLogic.run();

         boolean outside = true;
         ImPlot.setLegendLocation(ImPlotLocation.SouthWest, ImPlotOrientation.Horizontal, outside);

         boolean showingLegendPopup = false;
         for (ImPlotPlotLine plotLine : plotLines)
         {
            showingLegendPopup |= plotLine.render();
         }

         if (!plotLines.isEmpty() && ImPlot.isPlotHovered())
         {
            ImPlotPoint plotMousePosition = ImPlot.getPlotMousePos(ImPlotTools.IMPLOT_AUTO);
            int bufferIndex = (int) Math.round(plotMousePosition.getX());

            StringBuilder tooltipText = new StringBuilder();
            for (ImPlotPlotLine plotLine : plotLines)
            {
               if (!plotLine.getVariableName().isEmpty())
                  tooltipText.append(plotLine.getVariableName()).append(": ");
               tooltipText.append(plotLine.getValueString(bufferIndex)).append("\n");
            }
            ImGui.setTooltip(tooltipText.toString().trim());

            plotMousePosition.destroy();
         }

         if (popupContextWindowImGuiRenderer != null && !showingLegendPopup && ImGui.beginPopupContextWindow())
         {
            popupContextWindowImGuiRenderer.run();
            ImGui.endPopup();
         }

         if (dragAndDropPayloadConsumer != null)
         {
            if (ImPlot.beginDragDropTarget())
            {
               String payload = ImGui.acceptDragDropPayload(String.class);

               if (payload != null)
               {
                  dragAndDropPayloadConsumer.accept(payload);
               }
            }
         }
         ImPlot.endPlot();
      }
   }

   public void setDragAndDropPayloadConsumer(Consumer<String> dragAndDropPayloadConsumer)
   {
      this.dragAndDropPayloadConsumer = dragAndDropPayloadConsumer;
   }

   public void setPopupContextWindowImGuiRenderer(Runnable popupContextWindowImGuiRenderer)
   {
      this.popupContextWindowImGuiRenderer = popupContextWindowImGuiRenderer;
   }

   public ArrayList<ImPlotPlotLine> getPlotLines()
   {
      return plotLines;
   }

   public void queueRemovePlotLine(ImPlotPlotLine plotLineToRemove)
   {
      removalQueue.add(plotLineToRemove);
   }

   public int getFlags()
   {
      return flags;
   }

   public void setFlags(int flags)
   {
      this.flags = flags;
   }

   public void setFlag(int flag)
   {
      flags |= flag;
   }

   public void clearFlag(int flag)
   {
      flags &= ~flag;
   }

   public int getXFlags()
   {
      return xFlags;
   }

   public void setXFlags(int xFlags)
   {
      this.xFlags = xFlags;
   }

   public void setXFlag(int flag)
   {
      xFlags |= flag;
   }

   public void clearXFlag(int flag)
   {
      xFlags &= ~flag;
   }

   public int getYFlags()
   {
      return yFlags;
   }

   public void setYFlags(int yFlags)
   {
      this.yFlags = yFlags;
   }

   public void setYFlag(int flag)
   {
      yFlags |= flag;
   }

   public void clearYFlag(int flag)
   {
      yFlags &= ~flag;
   }

   public void setCustomDuringPlotLogic(Runnable customDuringPlotLogic)
   {
      this.customDuringPlotLogic = customDuringPlotLogic;
   }

   public void setCustomBeforePlotLogic(Runnable customBeforePlotLogic)
   {
      this.customBeforePlotLogic = customBeforePlotLogic;
   }
}
