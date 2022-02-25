package us.ihmc.gdx.ui.yo;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.*;
import imgui.internal.ImGui;
import us.ihmc.gdx.ui.tools.ImPlotTools;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ImPlotPlot
{
   private Consumer<String> dragAndDropPayloadConsumer;
   private Runnable popupContextWindowImGuiRenderer;
   private final ArrayList<ImPlotPlotLine> plotLines = new ArrayList<>();
   private final String xLabel;
   private final String yLabel;
   private final String label;
   private float graphHeight;
   private float graphWidth;
   private int flags;
   private int xFlags;
   private int yFlags;
   private final ImVec2 outerBoundsDimensionsPixels;

   public ImPlotPlot(String label)
   {
      this.label = label;
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

      graphHeight = 250;
      graphWidth = 100;

      outerBoundsDimensionsPixels = new ImVec2(graphWidth, graphHeight);
   }

   public void render()
   {
      graphWidth = ImGui.getColumnWidth();
      outerBoundsDimensionsPixels.x = graphWidth;

      if (ImPlot.beginPlot(label, xLabel, yLabel, outerBoundsDimensionsPixels, flags, xFlags, yFlags))
      {
         boolean outside = true;
         ImPlot.setLegendLocation(ImPlotLocation.SouthWest, ImPlotOrientation.Horizontal, outside);

         boolean showingLegendPopup = false;
         for (ImPlotPlotLine plotLine : plotLines)
         {
            showingLegendPopup |= plotLine.render();
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
}
