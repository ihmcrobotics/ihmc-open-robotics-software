package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class PlanarRegionFilteredMapUI
{
   private boolean captured = false;

   private ImGuiStoredPropertySetTuner mappingParametersTuner;
   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private PlanarRegionMappingManager mappingManager;
   private ImGuiPanel imGuiPanel;

   public PlanarRegionFilteredMapUI(String name, PlanarRegionMappingManager mappingManager)
   {
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      this.mappingManager = mappingManager;

      // TODO: Connect this to UI. JIRA Ticket HS-330 (https://jira.ihmc.us/browse/HS-330)
      //mappingParametersTuner = new ImGuiStoredPropertySetTuner(mappingManager.getFilteredMap().getParameters().getTitle());
      //mappingParametersTuner.create(mappingManager.getFilteredMap().getParameters());

      mapPlanarRegionsGraphic.generateMeshes(mappingManager.getMapRegions());
      mapPlanarRegionsGraphic.update();
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

      if(ImGui.button("Enable Live Mode"))
      {
         mappingManager.setEnableLiveMode(true);
      }
   }

   public void renderPlanarRegions()
   {
      if (mappingManager.getFilteredMap().isModified() && mappingManager.getMapRegions().getNumberOfPlanarRegions() > 0)
      {
         //LogTools.info("Regions Available and Modified: {} {}", mappingManager.getFilteredMap().isModified(), mappingManager.getMapRegions().getNumberOfPlanarRegions());
         mapPlanarRegionsGraphic.clear();
         mapPlanarRegionsGraphic.generateMeshes(mappingManager.getMapRegions());
         mapPlanarRegionsGraphic.update();
         mappingManager.getFilteredMap().setModified(false);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
         mapPlanarRegionsGraphic.getRenderables(renderables, pool);
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
