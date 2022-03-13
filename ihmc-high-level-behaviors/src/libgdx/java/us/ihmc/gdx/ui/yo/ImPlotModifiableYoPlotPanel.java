package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.simulation.scs2.GDXYoManager;

import java.util.ArrayList;

public class ImPlotModifiableYoPlotPanel extends ImGuiPanel
{
   private final ArrayList<ImPlotModifiableYoPlot> yoPlots = new ArrayList<>();
   private final ImPlotPlotPanelLayout layout = new ImPlotPlotPanelLayout();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiYoVariableSearchPanel yoVariableSearchPanel;
   private final GDXYoManager yoManager;
   private boolean update = false;

   public ImPlotModifiableYoPlotPanel(String panelName, ImGuiYoVariableSearchPanel yoVariableSearchPanel, GDXYoManager yoManager)
   {
      super(panelName, null, false, true);
      this.yoVariableSearchPanel = yoVariableSearchPanel;
      this.yoManager = yoManager;
      setRenderMethod(this::render);
      layout.setPlotRenderer(this::renderPlot, yoPlots::size);
   }

   private void renderPlot(int i)
   {
      yoPlots.get(i).render(layout.getPlotWidth(), layout.getPlotHeight(), update);
   }

   public void render()
   {
      ImGui.beginMenuBar();
      layout.renderLayoutMenu();
      if (ImGui.beginMenu(labels.get("Plots")))
      {
         if (ImGui.button("Add Plot"))
         {
            addPlot();
         }
         ImGui.endMenu();
      }

      ImGui.endMenuBar();

      layout.renderPlots();
   }

   public void setUpdate(boolean update)
   {
      this.update = update;
   }

   public ImPlotModifiableYoPlot addPlot()
   {
      ImPlotModifiableYoPlot imPlotModifiableYoPlot = new ImPlotModifiableYoPlot(yoVariableSearchPanel, this, yoManager);
      yoPlots.add(imPlotModifiableYoPlot);
      return imPlotModifiableYoPlot;
   }

   public ArrayList<ImPlotModifiableYoPlot> getYoPlots()
   {
      return yoPlots;
   }
}
