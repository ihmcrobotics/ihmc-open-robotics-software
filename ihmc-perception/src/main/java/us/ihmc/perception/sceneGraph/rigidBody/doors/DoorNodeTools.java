package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.YOLOv8.YOLOv8Node;

import static us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions.*;

public final class DoorNodeTools
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
               boolean doorHardwareComponent = switch (yoloNode.getDetection().objectClass())
               {
                  case DOOR_PULL_HANDLE, DOOR_KNOB, DOOR_PUSH_BAR, DOOR_LEVER -> true;
                  default -> false;
               };

               if (doorHardwareComponent)
               {
                  if (yoloNode.getChildren().isEmpty())
                  {
                     DoorNode doorNode = new DoorNode(sceneGraph.getNextID().getAndIncrement(), "Door");
                     doorNode.setOpeningMechanismTypeFromYoloClass(yoloNode.getDetection().objectClass());
                     modificationQueue.accept(new SceneGraphNodeAddition(doorNode, yoloNode));

                     SceneNode doorStaticNode = new StaticRelativeSceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                            "doorStaticHandle",
                                                                            sceneGraph.getIDToNodeMap(),
                                                                            doorNode.getID(),
                                                                            new RigidBodyTransform(),
                                                                            DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                                                            DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM,
                                                                            DOOR_YOLO_STATIC_MAXIMUM_DISTANCE_TO_LOCK_IN);
                     LogTools.info("Adding doorStaticHandle to scene graph.");
                     modificationQueue.accept(new SceneGraphNodeAddition(doorStaticNode, doorNode));
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
