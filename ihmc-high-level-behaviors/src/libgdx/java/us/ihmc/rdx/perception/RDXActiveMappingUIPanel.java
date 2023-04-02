package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.activeMapping.ActiveMappingModule;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class RDXActiveMappingUIPanel implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel imGuiPanel;

   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private ActiveMappingModule activeMappingModule;

   private final ImBoolean renderEnabled = new ImBoolean(true);

   public RDXActiveMappingUIPanel(String name, ActiveMappingModule mappingManager)
   {
      this.activeMappingModule = mappingManager;
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Render Enabled"), renderEnabled);

      if (ImGui.button("Calculate Footstep Plan") || ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
      {
         activeMappingModule.updateFootstepPlan();
         activeMappingModule.setWalkingEnabled(true);
      }
   }

   public void render3DGraphics()
   {
      synchronized (mapPlanarRegionsGraphic)
      {
         if(renderEnabled.get())
         {
            mapPlanarRegionsGraphic.generateMeshes(activeMappingModule.getPlanarRegionMap().getMapRegions());
            mapPlanarRegionsGraphic.update();
         }
      }
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         mapPlanarRegionsGraphic.getRenderables(renderables, pool);
      }
   }
}
