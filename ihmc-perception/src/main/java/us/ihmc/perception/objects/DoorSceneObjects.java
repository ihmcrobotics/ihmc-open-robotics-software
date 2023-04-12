package us.ihmc.perception.objects;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.scene.ROS2ArUcoDetectableObject;
import us.ihmc.perception.scene.SceneObjectAPI;
import us.ihmc.ros2.ROS2Topic;

public class DoorSceneObjects
{
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_FRAME = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_pull_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_PANEL = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_pull_door_panel");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_FRAME = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_push_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_PANEL = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_push_door_panel");

   public static final long PULL_DOOR_MARKER_ID = 0;
   public static final RigidBodyTransform PULL_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, -0.678702 - 0.005 - 0.006, 1.14141 + 0.02)
   );
   public static final RigidBodyTransform PULL_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, -0.678702, 1.14141)
   );
   public static final long PUSH_DOOR_MARKER_ID = 1;
   public static final RigidBodyTransform PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(0.0, 0.0, 0.0),
         new Point3D(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0,
                     -DoorModelParameters.ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Y_IN_PANEL_FRAME,
                     -DoorModelParameters.ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Z_IN_PANEL_FRAME)
   );
   public static final RigidBodyTransform PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER);
   static
   {
      PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER.getTranslation().add(0.0,
                                                               -DoorModelParameters.DOOR_FRAME_HINGE_OFFSET,
                                                               -DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);
   }

   public static ROS2ArUcoDetectableObject createPullDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      ROS2ArUcoDetectableObject sceneObject = new ROS2ArUcoDetectableObject("PullDoor0Panel");
      sceneObject.setupForROS2Publishing(ros2, DETECTED_PULL_DOOR_PANEL);
      return sceneObject;
   }

   public static ROS2ArUcoDetectableObject createPullDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      ROS2ArUcoDetectableObject sceneObject = new ROS2ArUcoDetectableObject("PullDoor0Frame");
      sceneObject.setupForROS2Publishing(ros2, DETECTED_PULL_DOOR_FRAME);
      return sceneObject;
   }

   public static ROS2ArUcoDetectableObject createPushDoorPanel(ROS2PublishSubscribeAPI ros2)
   {
      ROS2ArUcoDetectableObject sceneObject = new ROS2ArUcoDetectableObject("PushDoor0Panel");
      sceneObject.setupForROS2Publishing(ros2, DETECTED_PUSH_DOOR_PANEL);
      return sceneObject;
   }

   public static ROS2ArUcoDetectableObject createPushDoorFrame(ROS2PublishSubscribeAPI ros2)
   {
      ROS2ArUcoDetectableObject sceneObject = new ROS2ArUcoDetectableObject("PullDoor0Frame");
      sceneObject.setupForROS2Publishing(ros2, DETECTED_PUSH_DOOR_FRAME);
      return sceneObject;
   }
}
