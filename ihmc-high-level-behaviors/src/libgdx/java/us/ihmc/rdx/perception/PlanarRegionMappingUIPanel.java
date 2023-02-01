package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMappingParameters;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class PlanarRegionMappingUIPanel
{
   private ImGuiStoredPropertySetTuner mappingParametersTuner;
   private PlanarRegionMappingManager mappingManager;
   private ImGuiPanel imGuiPanel;
   private final ImBoolean liveModeEnabled = new ImBoolean();
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private boolean captured = false;
  
   public PlanarRegionMappingUIPanel(String name, PlanarRegionMappingManager mappingManager)
   {
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      this.mappingManager = mappingManager;

      mappingParametersTuner = new ImGuiStoredPropertySetTuner(mappingManager.getParameters().getTitle());
      mappingParametersTuner.create(mappingManager.getParameters());
      imGuiPanel.addChild(mappingParametersTuner);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Capture"))
      {
         mappingManager.setCaptured(true);
      }

      if (ImGui.button("Load Next Set"))
      {
         mappingManager.nextButtonCallback();
      }

      if (ImGui.button("Auto Increment"))
      {
         mappingManager.autoIncrementButtonCallback();
      }

      if (ImGui.checkbox("Enable Live Mode", liveModeEnabled))
      {
         mappingManager.setEnableLiveMode(liveModeEnabled.get());
      }
      ImGui.checkbox("Render live mode", renderEnabled);

      if (ImGui.button("Reset map"))
      {
         mappingManager.resetMap();
      }
      if (ImGui.button("Hard reset map"))
      {
         mappingManager.hardResetTheMap();
      }

      ImGui.checkbox("Show Parameter Tuners", mappingParametersTuner.getIsShowing());
   }

   public void setCaptured(boolean captured)
   {
      this.captured = captured;
   }

   public boolean isCaptured()
   {
      return captured;
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }
}
