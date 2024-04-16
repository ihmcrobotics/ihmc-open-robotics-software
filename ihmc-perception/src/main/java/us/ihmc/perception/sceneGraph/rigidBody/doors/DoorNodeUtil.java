package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;

public final class DoorNodeUtil
{
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
                     doorNode.setDoorHardwareTypeFromYoloClass(yoloNode.getDetectionClass());
                     modificationQueue.accept(new SceneGraphNodeAddition(doorNode, yoloNode));
                  }
                  else
                  {
                     for (SceneNode childNode : yoloNode.getChildren())
                     {
                        if (childNode instanceof DoorNode doorNode)
                        {
                           doorNode.setDoorHardwarePose(yoloNode.getObjectPose());
                        }
                     }
                  }
               }
            }
         }
      });
   }
}
