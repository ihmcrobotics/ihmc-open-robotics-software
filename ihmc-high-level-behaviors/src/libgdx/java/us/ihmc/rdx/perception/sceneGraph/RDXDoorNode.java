package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class RDXDoorNode extends RDXSceneNode
{
   private final DoorNode doorNode;
   private final ImGuiUniqueLabelMap labels;
   private final List<RDXDoorOpeningMechanismGraphic> openingMechanismGraphics = new ArrayList<>();
   private final RDXPlanarRegionsGraphic doorPanelPlanarRegionGraphic = new RDXPlanarRegionsGraphic();

   public RDXDoorNode(DoorNode doorNode, ImGuiUniqueLabelMap labels)
   {
      super(doorNode);
      this.doorNode = doorNode;
      this.labels = labels;
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      // Update door planar region graphic
      // We set a constant region ID just to get a consistent color in the planar region graphic
      doorNode.getDoorPanel().getPlanarRegion().setRegionId(2222);
      doorPanelPlanarRegionGraphic.setBlendOpacity(0.6f);
      doorPanelPlanarRegionGraphic.generateMeshes(new PlanarRegionsList(doorNode.getDoorPanel().getPlanarRegion()));
      doorPanelPlanarRegionGraphic.update();

//      if (doorOpeningMechanismInteractable != null)
//      {
//         doorOpeningMechanismVisualModelTransform.set(doorNode.getOpeningMechanismPose3D());
//         LibGDXTools.setDiffuseColor(doorOpeningMechanismInteractable.getModelInstance(), Color.WHITE); // TODO: keep?
//         doorOpeningMechanismInteractable.setPose(doorOpeningMechanismVisualModelTransform);
//      }
//      else
//      {
//         doorOpeningMechanismInteractable = createInteractableObject();
//      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (doorNode.getDoorPanel().getPlanarRegion().getArea() > 0.0)
      {
         doorPanelPlanarRegionGraphic.getRenderables(renderables, pool);
      }

//      if (doorOpeningMechanismInteractable != null && doorNode.getDoorPlanarRegion().getArea() > 0.0)
//      {
//         doorOpeningMechanismInteractable.getRenderables(renderables, pool);
//         doorPlanarRegionGraphic.getRenderables(renderables, pool);
//
//         // If the gizmo is NOT selected/enabled only then do we update the pose of the
//         // door opening mechanism interactable. We want the user to be able to move the interactable
//         // around if the gizmo is enabled
//         if (!doorOpeningMechanismInteractable.getSelectablePose3DGizmo().getSelected().get())
//         {
//            doorOpeningMechanismFrameGraphic.setPoseInWorldFrame(doorNode.getOpeningMechanismPose3D());
//            doorOpeningMechanismFrameGraphic.getRenderables(renderables, pool);
//         }
//      }
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
//
//      if (doorOpeningMechanismInteractable != null)
//      {
//         ImGui.sameLine();
//         ImGui.checkbox(labels.get("Show gizmo"), doorOpeningMechanismInteractable.getSelectablePose3DGizmo().getSelected());
//
//         if (doorOpeningMechanismInteractable.getSelectablePose3DGizmo().isSelected())
//         {
//            ImGui.text("Gizmo is enabled - the door opening mechanism pose will not update");
//         }
//      }
//
//      ImGui.text("Planar region info:");
//      ImGui.sameLine();
//      if (doorNode.getDoorPlanarRegion() != null && doorNode.getDoorPlanarRegion().getArea() > 0.0)
//      {
//         ImGui.text(doorNode.getDoorPlanarRegion().getDebugString());
//         ImGui.text("Last region update time: " + doorNode.getDoorPlanarRegionUpdateTime());
//      }
//      else
//      {
//         ImGui.text("N/A");
//      }
   }

   @Override
   public void destroy()
   {
      for (RDXDoorOpeningMechanismGraphic openingMechanismGraphic : openingMechanismGraphics)
         openingMechanismGraphic.destroy();

      doorPanelPlanarRegionGraphic.destroy();
   }
}
