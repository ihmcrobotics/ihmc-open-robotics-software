package us.ihmc.perception.objects;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.scene.ArUcoDetectableObject;

public class DoorSceneObjects
{
   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_ARUCO_MARKER_WIDTH = 0.2032;

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

   public static ArUcoDetectableObject createPullDoorPanel()
   {
      return new ArUcoDetectableObject("PullDoor0Panel", PULL_DOOR_MARKER_ID, DOOR_ARUCO_MARKER_WIDTH, PULL_DOOR_PANEL_TRANSFORM_TO_MARKER);
   }

   public static ArUcoDetectableObject createPullDoorFrame()
   {
      return new ArUcoDetectableObject("PullDoor0Frame", PULL_DOOR_MARKER_ID, DOOR_ARUCO_MARKER_WIDTH, PULL_DOOR_FRAME_TRANSFORM_TO_MARKER);
   }

   public static ArUcoDetectableObject createPushDoorPanel()
   {
      return new ArUcoDetectableObject("PushDoor0Panel", PUSH_DOOR_MARKER_ID, DOOR_ARUCO_MARKER_WIDTH, PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER);
   }

   public static ArUcoDetectableObject createPushDoorFrame()
   {
      return new ArUcoDetectableObject("PullDoor0Frame", PUSH_DOOR_MARKER_ID, DOOR_ARUCO_MARKER_WIDTH, PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER);
   }
}
