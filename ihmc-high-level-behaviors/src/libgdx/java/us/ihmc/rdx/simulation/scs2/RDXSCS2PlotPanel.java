package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.flag.ImGuiStyleVar;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.ArrayList;
import java.util.function.Consumer;

public class RDXSCS2PlotPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<ArrayList<RDXSCS2Plot>> plots = new ArrayList<>();
   private final ImInt rows = new ImInt(1);
   private final ImInt columns = new ImInt(1);
   private final RDXSCS2YoVariableSearchPanel searchPanel;
   private final RDXYoManager yoManager;
   private final Consumer<RDXSCS2PlotPanel> removeSelf;

   public RDXSCS2PlotPanel(String panelName, RDXSCS2YoVariableSearchPanel searchPanel, RDXYoManager yoManager, Consumer<RDXSCS2PlotPanel> removeSelf)
   {
      super(panelName, null, false, true);
      this.searchPanel = searchPanel;
      this.yoManager = yoManager;
      this.removeSelf = removeSelf;
      setRenderMethod(this::render);
      setRemovePadding(true);
      updatePlots();
   }

   public void render()
   {
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 8.0f, 8.0f);
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("Layout")))
      {
         ImGui.pushItemWidth(100);
         if (ImGui.inputInt(labels.get("Number of rows"), rows))
         {
            if (rows.get() < 1)
               rows.set(1);
            updatePlots();
         }
         if (ImGui.inputInt(labels.get("Number of columns"), columns))
         {
            if (columns.get() < 1)
               columns.set(1);
            updatePlots();
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
      if (ImGui.beginMenu(labels.get("Plots")))
      {
         if (ImGui.menuItem(labels.get("Remove this panel")))
         {
            removeSelf.accept(this);
         }
         ImGui.endMenu();
      }

      ImGui.endMenuBar();
      ImGui.popStyleVar();

      float plotWidth = ImGui.getColumnWidth() / columns.get();
      float plotHeight = ImGui.getContentRegionAvailY() / rows.get();

      ImGui.pushStyleVar(ImGuiStyleVar.ItemSpacing, 0.0f, 0.0f);
      ImGui.setCursorPosY(ImGui.getCursorPosY() + 1);
      for (int row = 0; row < rows.get(); row++)
      {
         for (int column = 0; column < columns.get(); column++)
         {
            plots.get(column).get(row).render(column, row, plotWidth, plotHeight);

            if (column != columns.get() - 1)
               ImGui.sameLine();
         }
      }
      ImGui.popStyleVar();
   }

   private void updatePlots()
   {
      while (plots.size() < columns.get())
         plots.add(new ArrayList<>());
      while (plots.size() > columns.get())
         plots.remove(plots.size() - 1);

      for (ArrayList<RDXSCS2Plot> column : plots)
      {
         while (column.size() < rows.get())
            column.add(new RDXSCS2Plot(searchPanel, this, yoManager));
         while (column.size() > rows.get())
            column.remove(column.size() - 1);
      }
   }

   public RDXSCS2Plot plot(int column, int row, String... variables)
   {
      RDXSCS2Plot plot = getPlot(column, row);
      for (String variable : variables)
         plot.addVariable(yoManager.getRootRegistry().findVariable(variable), false);
      return plot;
   }

   public RDXSCS2Plot getPlot(int column, int row)
   {
      return plots.get(column).get(row);
   }

   public ImInt getRows()
   {
      return rows;
   }

   public void setRows(int rows)
   {
      this.rows.set(rows);
      updatePlots();
   }

   public ImInt getColumns()
   {
      return columns;
   }

   public void setColumns(int columns)
   {
      this.columns.set(columns);
      updatePlots();
   }
}
