package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

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

   public void render()
   {
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("Plots")))
      {
         ImGui.inputInt(labels.get("Number of columns"), numberOfColumns);
         ImGui.inputInt(labels.get("Plot height"), plotHeight);
         ImGui.endMenu();
      }
      ImGui.endMenuBar();

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
