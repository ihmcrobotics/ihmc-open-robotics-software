package us.ihmc.gdx.ui.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.perception.PlanarRegionRegistration;

public class PlanarRegionSLAMPanel extends ImGuiPanel
{
   private ImInt index = new ImInt();

   private PlanarRegionRegistration planarRegionRegistration;

   public PlanarRegionSLAMPanel(String panelName)
   {
      super(panelName);
   }

   public void renderImGuiWidgets()
   {
      if(ImGui.button("Next"))
      {
         planarRegionRegistration.incrementIndex();
         planarRegionRegistration.update();
      }
   }

   public void setModule(PlanarRegionRegistration icp)
   {
      planarRegionRegistration = icp;
   }


}
