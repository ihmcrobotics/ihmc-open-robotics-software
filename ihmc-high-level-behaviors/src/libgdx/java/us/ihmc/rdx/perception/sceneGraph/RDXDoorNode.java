package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public class RDXDoorNode extends RDXSceneNode
{
   private final DoorNode doorNode;
   private final ImGuiUniqueLabelMap labels;
   private final Map<UUID, RDXDoorOpeningMechanismGraphic> openingMechanismGraphics = new HashMap<>();
   private final RDXPlanarRegionsGraphic doorPanelPlanarRegionGraphic = new RDXPlanarRegionsGraphic();

   public RDXDoorNode(DoorNode doorNode, ImGuiUniqueLabelMap labels)
   {
      super(doorNode);
      this.doorNode = doorNode;
      this.labels = labels;
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      // Update door planar region graphic
      // We set a constant region ID just to get a consistent color in the planar region graphic
      doorNode.getDoorPanel().getPlanarRegion().setRegionId(2222);
      doorPanelPlanarRegionGraphic.setBlendOpacity(0.6f);
      doorPanelPlanarRegionGraphic.generateMeshes(new PlanarRegionsList(doorNode.getDoorPanel().getPlanarRegion()));
      doorPanelPlanarRegionGraphic.update();

      for (DoorOpeningMechanism openingMechanism : doorNode.getOpeningMechanisms().values())
      {
         if (!openingMechanismGraphics.containsKey(openingMechanism.getDetectionID()))
         {
            RDXDoorOpeningMechanismGraphic graphic = new RDXDoorOpeningMechanismGraphic(openingMechanism);
            openingMechanismGraphics.put(openingMechanism.getDetectionID(), graphic);
         }

         openingMechanismGraphics.get(openingMechanism.getDetectionID()).update(new RigidBodyTransform(openingMechanism.getMechanismPose()));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (doorNode.getDoorPanel().getPlanarRegion().getArea() > 0.0)
      {
         doorPanelPlanarRegionGraphic.getRenderables(renderables, pool);
      }

      for (RDXDoorOpeningMechanismGraphic openingMechanismGraphic : openingMechanismGraphics.values())
      {
         openingMechanismGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      ImGui.text("Planar region info:");
      ImGui.sameLine();
      if (doorNode.getDoorPanel().getPlanarRegion().getArea() > 0.0)
      {
         ImGui.text(doorNode.getDoorPanel().getPlanarRegion().getDebugString());
         ImGui.text("Last region update time: " + doorNode.getDoorPanel().getPlanarRegionLastUpdateTimeMillis());
      }
      else
      {
         ImGui.text("N/A");
      }
   }

   @Override
   public void destroy()
   {
      for (RDXDoorOpeningMechanismGraphic openingMechanismGraphic : openingMechanismGraphics.values())
         openingMechanismGraphic.destroy();

      doorPanelPlanarRegionGraphic.destroy();
   }
}
