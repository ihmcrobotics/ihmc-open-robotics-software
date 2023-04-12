package us.ihmc.rdx.perception.scene.objects;

import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.objects.DoorSceneObjects;

/**
 * TODO: Separate the non RDX part into ihmc-perception.
 */
public class RDXDoorSceneCollection
{
   public static RDXSceneObject createPullDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                      "PullDoor%dPanel",
                                                      DoorSceneObjects.PULL_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, DoorSceneObjects.DETECTED_PULL_DOOR_PANEL);
      return sceneObject;
   }

   public static RDXSceneObject createPullDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                      "PullDoor%dFrame",
                                                      DoorSceneObjects.PULL_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, DoorSceneObjects.DETECTED_PULL_DOOR_FRAME);
      return sceneObject;
   }

   public static RDXSceneObject createPushDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                      "PushDoor%dPanel",
                                                      DoorSceneObjects.PUSH_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, DoorSceneObjects.DETECTED_PUSH_DOOR_PANEL);
      return sceneObject;
   }

   public static RDXSceneObject createPushDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      RDXSceneObject sceneObject = new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                      "PullDoor%dFrame",
                                                      DoorSceneObjects.PUSH_DOOR_MARKER_ID);
      sceneObject.setupForROS2Updating(ros2, DoorSceneObjects.DETECTED_PUSH_DOOR_FRAME);
      return sceneObject;
   }
}
