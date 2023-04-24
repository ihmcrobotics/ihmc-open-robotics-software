package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticArUcoRelativeDetectableSceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class DoorSceneNodeDefinitions
{
   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_ARUCO_MARKER_WIDTH = 0.2032;

   public static final int PULL_DOOR_MARKER_ID = 0;
   public static final int PUSH_DOOR_MARKER_ID = 1;

   public static final RigidBodyTransform PULL_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getRotation(), 180.0, 0.0, 0.0);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setX(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET
                                                                + DoorModelParameters.ARUCO_MARKER_PULL_SIDE_BOTTOM_RIGHT_CORNER_Y_IN_PANEL_FRAME);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setZ(DoorModelParameters.ARUCO_MARKER_PULL_SIDE_BOTTOM_RIGHT_CORNER_Z_IN_PANEL_FRAME);
   }
   /** The frame has the same origin as the panel. */
   public static final RigidBodyTransform PULL_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER);

   public static final RigidBodyTransform PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setX(DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET
                                                                + DoorModelParameters.ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Y_IN_PANEL_FRAME);
      PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setZ(DoorModelParameters.ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Z_IN_PANEL_FRAME);
   }
   public static final RigidBodyTransform PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER);
   static
   {
      PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER.getTranslation().addZ(-DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);
   }

   public static final double DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN = 3.0;

   public static final RigidBodyTransform PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getTranslation().add(0.03, 0.09, -0.105);
      EuclidCoreMissingTools.setYawPitchRollDegrees(PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getRotation(), 90.0, 0.0, 90.0);
   }

   public static final RigidBodyTransform PUSH_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      PUSH_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getTranslation().add(0.03, 0.09, -0.105);
      EuclidCoreMissingTools.setYawPitchRollDegrees(PUSH_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getRotation(), 90.0, 0.0, 90.0);
   }

   // TODO: These transforms need to be verified.
   public static final String DOOR_PANEL_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorPanel/DoorPanel.g3dj";
   public static final RigidBodyTransform PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform PUSH_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_FRAME_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorFrame/DoorFrame.g3dj";
   public static final RigidBodyTransform PULL_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj";
   public static final RigidBodyTransform PULL_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static ArUcoDetectableNode createPullDoorPanel()
   {
      return new ArUcoDetectableNode("PullDoorPanel",
                                     PULL_DOOR_MARKER_ID,
                                     DOOR_ARUCO_MARKER_WIDTH,
                                     PULL_DOOR_PANEL_TRANSFORM_TO_MARKER,
                                     DOOR_PANEL_VISUAL_MODEL_FILE_PATH,
                                     PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }

   public static ArUcoDetectableNode createPushDoorPanel()
   {
      return new ArUcoDetectableNode("PushDoorPanel",
                                     PUSH_DOOR_MARKER_ID,
                                     DOOR_ARUCO_MARKER_WIDTH,
                                     PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER,
                                     DOOR_PANEL_VISUAL_MODEL_FILE_PATH,
                                     PUSH_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }

   public static StaticArUcoRelativeDetectableSceneNode createPullDoorFrame()
   {
      return new StaticArUcoRelativeDetectableSceneNode("PullDoorFrame",
                                                        PULL_DOOR_MARKER_ID,
                                                        DOOR_ARUCO_MARKER_WIDTH,
                                                        PULL_DOOR_FRAME_TRANSFORM_TO_MARKER,
                                                        DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                        PULL_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                        DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN);
   }

   public static StaticArUcoRelativeDetectableSceneNode createPushDoorFrame()
   {
      return new StaticArUcoRelativeDetectableSceneNode("PullDoorFrame",
                                                        PUSH_DOOR_MARKER_ID,
                                                        DOOR_ARUCO_MARKER_WIDTH,
                                                        PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER,
                                                        DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                        PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                        DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN);
   }

   public static ArUcoDetectableNode createPushDoorLeverHandle()
   {
      return new ArUcoDetectableNode("PushDoorLeverHandle",
                                     PUSH_DOOR_MARKER_ID,
                                     DOOR_ARUCO_MARKER_WIDTH,
                                     PUSH_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER,
                                     DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                     PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }

   public static ArUcoDetectableNode createPullDoorLeverHandle()
   {
      return new ArUcoDetectableNode("PullDoorLeverHandle",
                                     PULL_DOOR_MARKER_ID,
                                     DOOR_ARUCO_MARKER_WIDTH,
                                     PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER,
                                     DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                     PULL_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }
}
