package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiPanel;

import java.util.ArrayList;

public class ImPlotYoPlotPanel extends ImGuiPanel
{
   private final ArrayList<ImPlotYoPlot> yoPlots = new ArrayList<>();
   private final ImPlotPlotPanelLayout layout = new ImPlotPlotPanelLayout();
   private boolean update = false;

   public ImPlotYoPlotPanel(String panelName)
   {
      super(panelName, null, false, true);
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
      ImGui.endMenuBar();

      layout.renderPlots();
   }

   public void setUpdate(boolean update)
   {
      this.update = update;
   }

   public ArrayList<ImPlotYoPlot> getYoPlots()
   {
      return yoPlots;
   }
}
