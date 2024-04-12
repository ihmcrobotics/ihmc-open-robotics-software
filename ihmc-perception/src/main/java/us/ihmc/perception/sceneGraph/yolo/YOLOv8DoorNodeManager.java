package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;

/**
 * Manages adding door nodes to children of YOLOv8Node\s
 */
public class YOLOv8DoorNodeManager
{
   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
         {
            boolean doorComponent = false;

            if (sceneNode instanceof YOLOv8Node yoloNode)
            {
               switch (yoloNode.getDetectionClass())
               {
                  case DOOR_PULL_HANDLE:
                  case DOOR_KNOB:
                  case DOOR_PUSH_BAR:
                  case DOOR_LEVER:
                     doorComponent = true;
               }
            }

            if (doorComponent)
            {
               System.out.println("was a door node");
            }
         }
      });
   }
}
