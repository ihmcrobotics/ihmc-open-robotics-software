package us.ihmc.rdx.perception.scene.objects;

import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.ArUcoObjectsPerceptionManager;

/**
 * TODO: Separate the non RDX part into ihmc-perception.
 */
public class DoorSceneCollection
{
   public static RDXSceneObject createPullDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                      "PullDoor%dFrame",
                                                      ArUcoObjectsPerceptionManager.PULL_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, ArUcoObjectsPerceptionManager.DETECTED_PULL_DOOR_FRAME);
      return sceneObject;
   }

   public static RDXSceneObject createPullDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                      "PullDoor%dPanel",
                                                      ArUcoObjectsPerceptionManager.PULL_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, ArUcoObjectsPerceptionManager.DETECTED_PULL_DOOR_PANEL);
      return sceneObject;
   }

   public static RDXSceneObject createPushDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                      "PullDoor%dFrame",
                                                      ArUcoObjectsPerceptionManager.PUSH_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, ArUcoObjectsPerceptionManager.DETECTED_PUSH_DOOR_FRAME);
      return sceneObject;
   }

   public static RDXSceneObject createPushDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                      "PushDoor%dPanel",
                                                      ArUcoObjectsPerceptionManager.PUSH_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, ArUcoObjectsPerceptionManager.DETECTED_PUSH_DOOR_PANEL);
      return sceneObject;
   }
}
