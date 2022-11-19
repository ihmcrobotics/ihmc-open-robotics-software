package us.ihmc.rdx.ui.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.perception.PlanarRegionRegistration;

public class PlanarRegionRegistrationPanel extends ImGuiPanel
{
   private ImInt index = new ImInt();

   private PlanarRegionRegistration planarRegionRegistration;

   public PlanarRegionRegistrationPanel(String panelName, PlanarRegionRegistration icp)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      planarRegionRegistration = icp;
   }

   public void renderImGuiWidgets()
   {
      if(ImGui.button("Next"))
      {
         planarRegionRegistration.incrementIndex();
         planarRegionRegistration.update();
      }
   }

}
