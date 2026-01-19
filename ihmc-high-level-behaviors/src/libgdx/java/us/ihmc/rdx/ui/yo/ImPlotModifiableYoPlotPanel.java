package us.ihmc.rdx.ui.yo;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ImPlotModifiableYoPlotPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<ArrayList<ImPlotModifiableYoPlot>> yoPlots = new ArrayList<>();
   private final ImInt rows = new ImInt(1);
   private final ImInt columns = new ImInt(1);
   private final ImInt plotHeight = new ImInt(60);
   private float plotWidth;
   private final ImGuiYoVariableSearchPanel yoVariableSearchPanel;
   private final RDXYoManager yoManager;
   private final Consumer<ImPlotModifiableYoPlotPanel> removeSelf;

   public ImPlotModifiableYoPlotPanel(String panelName,
                                      ImGuiYoVariableSearchPanel yoVariableSearchPanel,
                                      RDXYoManager yoManager,
                                      Consumer<ImPlotModifiableYoPlotPanel> removeSelf)
   {
      super(panelName, null, false, true);
      this.yoVariableSearchPanel = yoVariableSearchPanel;
      this.yoManager = yoManager;
      this.removeSelf = removeSelf;
      setRenderMethod(this::render);
      updatePlots();
   }

   private void updatePlots()
   {
      while (yoPlots.size() < columns.get())
         yoPlots.add(new ArrayList<>());
      while (yoPlots.size() > columns.get())
         yoPlots.remove(yoPlots.size() - 1);

      for (ArrayList<ImPlotModifiableYoPlot> column : yoPlots)
      {
         while (column.size() < rows.get())
            column.add(new ImPlotModifiableYoPlot(yoVariableSearchPanel, this, yoManager));
         while (column.size() > rows.get())
            column.remove(column.size() - 1);
      }
   }

   public void render()
   {
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
         if (ImGui.inputInt(labels.get("Plot height"), plotHeight, 10))
         {
            if (plotHeight.get() < 10)
               plotHeight.set(10);
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

      plotWidth = ImGui.getColumnWidth() / columns.get();

      for (int row = 0; row < rows.get(); row++)
      {
         for (int column = 0; column < columns.get(); column++)
         {
            yoPlots.get(column).get(row).render(plotWidth, plotHeight.get());

            if (column != columns.get() - 1)
               ImGui.sameLine();
         }
      }
   }

   public ImPlotModifiableYoPlot plot(int column, int row, String... variables)
   {
      ImPlotModifiableYoPlot plot = getPlot(column, row);
      for (String variable : variables)
         plot.addVariable(yoManager.getRootRegistry().findVariable(variable), false);
      return plot;
   }

   public ImPlotModifiableYoPlot getPlot(int column, int row)
   {
      return yoPlots.get(column).get(row);
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
