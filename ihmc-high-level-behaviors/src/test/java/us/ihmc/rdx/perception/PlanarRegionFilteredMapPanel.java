package us.ihmc.rdx.perception;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiPanel;

public class PlanarRegionFilteredMapPanel extends ImGuiPanel
{
   private boolean captured = false;
   private PlanarRegionMappingManager manager;

   public PlanarRegionFilteredMapPanel(String panelName, PlanarRegionMappingManager manager)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      this.manager = manager;
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Capture"))
      {
         captured = true;
      }

      if (ImGui.button("Load Next Set"))
      {
         manager.nextButtonCallback();
      }
   }

   public void setCaptured(boolean captured)
   {
      this.captured = captured;
   }

   public boolean isCaptured()
   {
      return captured;
   }
}
