package us.ihmc.rdx.ui.yo;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ImPlotModifiableYoPlotPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<ImPlotModifiableYoPlot> yoPlots = new ArrayList<>();
   private final ImInt numberOfRows = new ImInt(1);
   private final ImInt numberOfColumns = new ImInt(1);
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
   }

   public void render()
   {
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("Layout")))
      {
         ImGui.pushItemWidth(100);
         if (ImGui.inputInt(labels.get("Number of rows"), numberOfRows))
         {
            if (numberOfRows.get() < 1)
               numberOfRows.set(1);
         }
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
      if (ImGui.beginMenu(labels.get("Plots")))
      {
         if (ImGui.button(labels.get("Add Plot")))
         {
            addPlot();
         }
         if (ImGui.menuItem(labels.get("Remove this panel")))
         {
            removeSelf.accept(this);
         }
         ImGui.endMenu();
      }

      ImGui.endMenuBar();

      plotWidth = ImGui.getColumnWidth() / numberOfColumns.get();

      for (int i = 0; i < yoPlots.size(); i++)
      {
         yoPlots.get(i).render(plotWidth, plotHeight.get());

         if (i % numberOfColumns.get() != numberOfColumns.get() - 1)
            ImGui.sameLine();
      }
   }

   public ImPlotModifiableYoPlot addPlot(String... variables)
   {
      ImPlotModifiableYoPlot plot = addPlot();
      for (String variable : variables)
         plot.addVariable(yoManager.getRootRegistry().findVariable(variable), false);
      return plot;
   }

   public ImPlotModifiableYoPlot addPlot()
   {
      ImPlotModifiableYoPlot imPlotModifiableYoPlot = new ImPlotModifiableYoPlot(yoVariableSearchPanel, this, yoManager, this::removePlot);
      yoPlots.add(imPlotModifiableYoPlot);
      return imPlotModifiableYoPlot;
   }

   private void removePlot(ImPlotModifiableYoPlot plot)
   {
      yoPlots.remove(plot);
   }

   public ArrayList<ImPlotModifiableYoPlot> getYoPlots()
   {
      return yoPlots;
   }

   public void setNumberOfRows(int numberOfRows)
   {
      this.numberOfRows.set(numberOfRows);
   }

   public void setNumberOfColumns(int numberOfColumns)
   {
      this.numberOfColumns.set(numberOfColumns);
   }
}
