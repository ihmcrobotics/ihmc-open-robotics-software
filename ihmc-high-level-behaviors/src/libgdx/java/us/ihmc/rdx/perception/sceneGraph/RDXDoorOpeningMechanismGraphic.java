package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;

import javax.annotation.Nullable;

public class RDXDoorOpeningMechanismGraphic
{
   private final DoorOpeningMechanism openingMechanism;

   public RDXDoorOpeningMechanismGraphic(DoorOpeningMechanism openingMechanism)
   {
      this.openingMechanism = openingMechanism;
   }

   @Nullable
   private RDXInteractableObject interactableObject;
   private final RigidBodyTransform visualModelTransform = new RigidBodyTransform();
   private final RDXReferenceFrameGraphic frameGraphic = new RDXReferenceFrameGraphic(0.2);

   private RDXInteractableObject createInteractableObject()
   {
      RDXInteractableObject interactableObject = null;

      switch (openingMechanism.getType())
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
         case KNOB ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
      }

      // Disable the gizmo by default
      if (interactableObject != null)
         interactableObject.getSelectablePose3DGizmo().setSelected(false);

      return interactableObject;
   }

   public void destroy()
   {
      if (interactableObject != null)
      {
         interactableObject.getModelInstance().model.dispose();
         interactableObject.clear();
      }

      frameGraphic.dispose();
   }
}
