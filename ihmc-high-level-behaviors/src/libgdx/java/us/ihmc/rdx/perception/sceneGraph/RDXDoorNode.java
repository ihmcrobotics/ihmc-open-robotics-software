package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorHardwareType;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

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

   private final transient PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final transient PlanarRegion lastDoorRegion = new PlanarRegion();

   // TODO: remove these
   private static final int SWITCH_SIDE_THRESHOLD = 10;
   private int switchSide = 0;
   private RobotSide lastSide;

   public RDXDoorNode(DoorNode yoloDoorNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloDoorNode);
      this.doorNode = yoloDoorNode;
      this.labels = labels;

      doorPlanarRegionGraphic.setBlendOpacity(0.6f);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      // Update door planar region graphic
      if (!lastDoorRegion.epsilonEquals(doorNode.getDoorPlanarRegion(), 0.1))
      {
         planarRegionsList.clear();
         doorNode.getDoorPlanarRegion().setRegionId(2222);
         planarRegionsList.addPlanarRegion(doorNode.getDoorPlanarRegion());
         doorPlanarRegionGraphic.generateMeshes(planarRegionsList);

         lastDoorRegion.set(doorNode.getDoorPlanarRegion());
      }

      doorPlanarRegionGraphic.update();

      if (interactableObject != null)
      {
         Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(doorNode.getDoorPlanarRegion());

         Line2D doorLineNormal = new Line2D(planarRegionCentroidInWorld.getX(),
                                            planarRegionCentroidInWorld.getY(),
                                            doorNode.getDoorPlanarRegion().getNormalX(),
                                            doorNode.getDoorPlanarRegion().getNormalY());
         Point2D doorLeverPointInWorld2D = new Point2D(doorNode.getObjectPose().getTranslation());

         RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(doorLeverPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;

         if (lastSide == null)
            lastSide = doorSide;

         // Glitch filter
         if (lastSide != doorSide)
         {
            if (++switchSide > SWITCH_SIDE_THRESHOLD)
            {
               // Switch sides
               switchSide = 0;
               LogTools.info("Door lever switched sides");
            }
            else
            {
               doorSide = lastSide;
            }
         }

         double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());

         if (doorNode.getDoorHardwareType() == DoorHardwareType.LEVER_HANDLE)
            visualModelTransformToWorld.getRotation().setYawPitchRoll(yaw, 0.0, doorSide == RobotSide.LEFT ? Math.PI : 0.0);
         else
            visualModelTransformToWorld.getRotation().setToYawOrientation(yaw);

         visualModelTransformToWorld.getTranslation().set(doorNode.getObjectPose().getTranslation());

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
      if (!planarRegionsList.isEmpty())
         ImGui.text(planarRegionsList.getPlanarRegion(0).getDebugString());
      else
         ImGui.text("N/A");

      ImGui.text("Has planar region: " + !planarRegionsList.isEmpty());
   }

   @Override
   public void destroy()
   {
      if (interactableObject != null)
      {
         interactableObject.getModelInstance().model.dispose();
         interactableObject.clear();
      }
   }

   private RDXInteractableObject createInteractableObject()
   {
      RDXInteractableObject interactableObject = null;

      switch (doorNode.getDoorHardwareType())
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
