package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import javax.annotation.Nullable;
import java.util.Set;

public class RDXDoorNode extends RDXSceneNode
{
   private final DoorNode doorNode;
   private final ImGuiUniqueLabelMap labels;
   @Nullable
   private RDXInteractableObject interactableObject;
   private final RigidBodyTransform visualModelTransformToWorld = new RigidBodyTransform();
   private final RDXPlanarRegionsGraphic doorPlanarRegionGraphic = new RDXPlanarRegionsGraphic();
   private final RDXReferenceFrameGraphic doorOpeningMechanismFrameGraphic = new RDXReferenceFrameGraphic(0.2);

   public RDXDoorNode(DoorNode yoloDoorNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloDoorNode);
      this.doorNode = yoloDoorNode;
      this.labels = labels;

      doorPlanarRegionGraphic.setBlendOpacity(0.6f);
      doorOpeningMechanismFrameGraphic.setPoseInWorldFrame(doorNode.getOpeningMechanismPose3D());
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      // Update door planar region graphic
      // We set a constant region ID just to get a consistent color in the planar region graphic
      doorNode.getDoorPlanarRegion().setRegionId(2222);
      doorPlanarRegionGraphic.generateMeshes(new PlanarRegionsList(doorNode.getDoorPlanarRegion()));
      doorPlanarRegionGraphic.update();

      if (interactableObject != null)
      {
         visualModelTransformToWorld.set(doorNode.getOpeningMechanismPose3D());
         LibGDXTools.setDiffuseColor(interactableObject.getModelInstance(), Color.WHITE); // TODO: keep?
         interactableObject.setPose(visualModelTransformToWorld);
      }
      else
      {
         interactableObject = createInteractableObject();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (interactableObject != null)
         interactableObject.getRenderables(renderables, pool);

      doorPlanarRegionGraphic.getRenderables(renderables, pool);

      doorOpeningMechanismFrameGraphic.setPoseInWorldFrame(doorNode.getOpeningMechanismPose3D());
      doorOpeningMechanismFrameGraphic.getRenderables(renderables, pool);
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      if (interactableObject != null)
      {
         ImGui.sameLine();
         ImGui.checkbox(labels.get("Show gizmo"), interactableObject.getSelectablePose3DGizmo().getSelected());
      }

      ImGui.text("Planar region info:");
      ImGui.sameLine();
      if (doorNode.getDoorPlanarRegion() != null && doorNode.getDoorPlanarRegion().getArea() > 0.0)
      {
         ImGui.text(doorNode.getDoorPlanarRegion().getDebugString());
         ImGui.text("Last region update time: " + doorNode.getDoorPlanarRegionUpdateTime());
      }
      else
      {
         ImGui.text("N/A");
      }
   }

   @Override
   public void destroy()
   {
      if (interactableObject != null)
      {
         interactableObject.getModelInstance().model.dispose();
         interactableObject.clear();
      }

      doorPlanarRegionGraphic.destroy();
      doorOpeningMechanismFrameGraphic.dispose();
   }

   private RDXInteractableObject createInteractableObject()
   {
      RDXInteractableObject interactableObject = null;

      switch (doorNode.getOpeningMechanismType())
      {
         case LEVER_HANDLE ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case PULL_HANDLE ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case PUSH_BAR ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
      }

      return interactableObject;
   }
}
