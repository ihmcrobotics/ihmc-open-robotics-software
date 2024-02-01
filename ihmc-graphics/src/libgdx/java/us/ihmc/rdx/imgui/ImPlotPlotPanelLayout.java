package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.function.IntConsumer;
import java.util.function.IntSupplier;

public class ImPlotPlotPanelLayout
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt numberOfColumns = new ImInt(1);
   private final ImInt plotHeight = new ImInt(60);
   private float plotWidth;
   private IntConsumer plotRenderer;
   private IntSupplier sizeSupplier;

   public void setPlotRenderer(IntConsumer plotRenderer, IntSupplier sizeSupplier)
   {
      this.plotRenderer = plotRenderer;
      this.sizeSupplier = sizeSupplier;
   }

   public void renderLayoutMenu()
   {
      if (ImGui.beginMenu(labels.get("Layout")))
      {
         ImGui.pushItemWidth(100);
         if (ImGui.inputInt(labels.get("Number of columns"), numberOfColumns))
         {
            if (numberOfColumns.get() < 1)
               numberOfColumns.set(1);
         }
         if (ImGui.inputInt(labels.get("Plot height"), plotHeight))
         {
            if (plotHeight.get() < 10)
               plotHeight.set(10);
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
   }

   public void renderPlots()
   {
      int numberOfColumnsLocal = numberOfColumns.get();
      plotWidth = ImGui.getColumnWidth() / numberOfColumnsLocal;

      for (int i = 0; i < sizeSupplier.getAsInt(); i++)
      {
         plotRenderer.accept(i);

         if (i % numberOfColumnsLocal != numberOfColumnsLocal - 1)
            ImGui.sameLine();
      }
   }

   public int getNumberOfColumns()
   {
      return numberOfColumns.get();
   }

   public int getPlotHeight()
   {
      return plotHeight.get();
   }

   public float getPlotWidth()
   {
      return plotWidth;
   }
}
