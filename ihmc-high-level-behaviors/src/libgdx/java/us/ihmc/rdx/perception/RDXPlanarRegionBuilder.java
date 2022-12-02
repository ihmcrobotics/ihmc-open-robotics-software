package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.ArrayList;
import java.util.Set;

public class RDXPlanarRegionBuilder
{
   private ArrayList<RDXEditablePlanarRegion> editableRegions = new ArrayList<>();
   private RDXEditablePlanarRegion selectedRegion;
   private RDX3DPanel panel3D;

   public RDXPlanarRegionBuilder(RDX3DPanel panel3D)
   {
      this.panel3D = panel3D;
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewPick);
      panel3D.getScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
      selectedRegion = null;
      for (RDXEditablePlanarRegion editableRegion : editableRegions)
      {
         if (editableRegion.getOriginGizmo().isSelected())
         {
            selectedRegion = editableRegion;
         }
      }
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXEditablePlanarRegion editableRegion : editableRegions)
      {
         editableRegion.calculate3DViewPick(input);
      }
   }

   private void process3DViewPick(ImGui3DViewInput input)
   {
      for (RDXEditablePlanarRegion editableRegion : editableRegions)
      {
         editableRegion.process3DViewInput(input);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Create planar region"))
      {
         RDXEditablePlanarRegion editablePlanarRegion = new RDXEditablePlanarRegion(panel3D);
         editablePlanarRegion.getOriginGizmo().getSelected().set(true);
         editableRegions.add(editablePlanarRegion);
      }
      if (selectedRegion == null)
         ImGui.beginDisabled();
      if (ImGui.button("Add vertices"))
      {
         selectedRegion.addVertex();
      }
      if (selectedRegion == null)
         ImGui.endDisabled();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXEditablePlanarRegion editableRegion : editableRegions)
      {
         editableRegion.getRenderables(renderables, pool, sceneLevels);
      }
   }
}
