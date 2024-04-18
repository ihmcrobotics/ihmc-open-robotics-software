package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;

public final class DoorNodeUtil
{
   /**
    * Creates new door nodes as children of {@link YOLOv8Node}\s which are door components.
    * If a door node already exists as a child of a YOLO node which is a door component, we update
    * the pose of the door component in the door node with the detection pose from the YOLO node.
    * @param sceneGraph the SceneGraph
    */
   public static void addDoorNodes(SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
         {
            if (sceneNode instanceof YOLOv8Node yoloNode)
            {
               boolean doorHardwareComponent = switch (yoloNode.getDetectionClass())
               {
                  case DOOR_PULL_HANDLE, DOOR_KNOB, DOOR_PUSH_BAR, DOOR_LEVER -> true;
                  default -> false;
               };

               if (doorHardwareComponent)
               {
                  if (yoloNode.getChildren().isEmpty())
                  {
                     DoorNode doorNode = new DoorNode(sceneGraph.getNextID().getAndIncrement(), "Door");
                     doorNode.setOpeningMechanismTypeFromYoloClass(yoloNode.getDetectionClass());
                     modificationQueue.accept(new SceneGraphNodeAddition(doorNode, yoloNode));
                  }
                  else
                  {
                     for (SceneNode childNode : yoloNode.getChildren())
                     {
                        if (childNode instanceof DoorNode doorNode)
                        {
                           doorNode.setOpeningMechanismPoint3D(yoloNode.getObjectPose().getTranslation());
                        }
                     }
                  }
               }
            }
         }
      });
   }
}
