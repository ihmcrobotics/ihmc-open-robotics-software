package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

import java.util.ArrayList;

public class ImPlotPlotPanel extends ImGuiPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<ImPlotPlot> plots = new ArrayList<>();
   private final ImInt numberOfColumns = new ImInt(1);
   private final ImInt plotHeight = new ImInt(60);

   public ImPlotPlotPanel(String panelName)
   {
      super(panelName, null, false, true);

      setRenderMethod(this::render);
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

      int numberOfColumns = this.numberOfColumns.get();
      float plotWidth = ImGui.getColumnWidth() / numberOfColumns;

      for (int i = 0; i < plots.size(); i++)
      {
         plots.get(i).render(plotWidth, plotHeight.get());

         if (i % numberOfColumns != numberOfColumns - 1)
            ImGui.sameLine();
      }
   }

   public ArrayList<ImPlotPlot> getPlots()
   {
      return plots;
   }
}
