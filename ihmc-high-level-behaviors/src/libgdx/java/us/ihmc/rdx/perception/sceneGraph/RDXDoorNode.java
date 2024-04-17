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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.doors.OpeningMechanismType;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
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

   private static final int DOOR_LEVER_SWITCH_SIDE_THRESHOLD = 10;
   private transient int doorLeverSwitchSide = 0;
   private transient RobotSide doorLeverLastSide;

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
      Point3D currentCameraFocus = new Point3D(doorNode.getOpeningMechanismPose().getTranslation());

      if (interpolatedFocus.getX() == 0)
      {
         interpolatedFocus.set(currentCameraFocus);
      }

      interpolatedFocus.interpolate(currentCameraFocus, 0.01);

      RDXBaseUI.getInstance().getPrimary3DPanel().getCamera3D().setCameraFocusPoint(interpolatedFocus);

      // Update door planar region graphic
      doorNode.getDoorPlanarRegion().setRegionId(2222);
      doorPlanarRegionGraphic.generateMeshes(new PlanarRegionsList(doorNode.getDoorPlanarRegion()));

      doorPlanarRegionGraphic.update();

      if (interactableObject != null)
      {
         Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(doorNode.getDoorPlanarRegion());

         Line2D doorLineNormal = new Line2D(planarRegionCentroidInWorld.getX(),
                                            planarRegionCentroidInWorld.getY(),
                                            doorNode.getDoorPlanarRegion().getNormalX(),
                                            doorNode.getDoorPlanarRegion().getNormalY());
         Point2D openingMechanismPointInWorld2D = new Point2D(doorNode.getOpeningMechanismPose().getTranslation());

         RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(openingMechanismPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;

         if (doorNode.getOpeningMechanismType() == OpeningMechanismType.LEVER_HANDLE)
         {
            if (doorLeverLastSide == null)
               doorLeverLastSide = doorSide;

            // Glitch filter
            if (doorLeverLastSide != doorSide)
            {
               if (++doorLeverSwitchSide > DOOR_LEVER_SWITCH_SIDE_THRESHOLD)
               {
                  // Switch sides
                  doorLeverSwitchSide = 0;
                  LogTools.info("Door lever switched sides");
               }
               else
               {
                  doorSide = doorLeverLastSide;
               }
            }
         }

         double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());

         if (doorNode.getOpeningMechanismType() == OpeningMechanismType.LEVER_HANDLE)
            visualModelTransformToWorld.getRotation().setYawPitchRoll(yaw, 0.0, doorSide == RobotSide.LEFT ? Math.PI : 0.0);
         else
            visualModelTransformToWorld.getRotation().setToYawOrientation(yaw);

         visualModelTransformToWorld.getTranslation().set(doorNode.getOpeningMechanismPose().getTranslation());

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

   Point3D interpolatedFocus = new Point3D();

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
