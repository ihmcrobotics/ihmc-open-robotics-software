package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8DoorNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;

public class RDXYOLOv8DoorNode extends RDXYOLOv8Node
{
   private final YOLOv8DoorNode yoloDoorNode;

   public RDXYOLOv8DoorNode(YOLOv8DoorNode yoloDoorNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloDoorNode, labels);
      this.yoloDoorNode = yoloDoorNode;
   }

   @Override
   public RDXInteractableObject createInteractableObject()
   {
      RDXInteractableObject interactableObject = null;

      switch (yoloDoorNode.getDetectionClass())
      {
         case DOOR_LEVER ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case DOOR_PULL_HANDLE ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case DOOR_PUSH_BAR ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
      }

      return interactableObject;
   }
}
