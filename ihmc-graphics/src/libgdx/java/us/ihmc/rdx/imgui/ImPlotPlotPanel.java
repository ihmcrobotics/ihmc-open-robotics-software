package us.ihmc.rdx.imgui;

import imgui.ImGui;

import java.util.ArrayList;

public class ImPlotPlotPanel extends RDXPanel
{
   private final ArrayList<ImPlotPlot> plots = new ArrayList<>();
   private final ImPlotPlotPanelLayout layout = new ImPlotPlotPanelLayout();

   public ImPlotPlotPanel(String panelName)
   {
      super(panelName, null, false, true);
      setRenderMethod(this::render);
      layout.setPlotRenderer(this::renderPlot, plots::size);
   }

   private void renderPlot(int i)
   {
      plots.get(i).render(layout.getPlotWidth(), layout.getPlotHeight());
   }

   public void render()
   {
      ImGui.beginMenuBar();
      layout.renderLayoutMenu();
      ImGui.endMenuBar();

      layout.renderPlots();
   }

   public ArrayList<ImPlotPlot> getPlots()
   {
      return plots;
   }
}
