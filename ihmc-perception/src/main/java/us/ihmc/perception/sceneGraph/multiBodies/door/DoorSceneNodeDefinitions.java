package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticArUcoRelativeDetectableSceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * We want to measure this stuff in the right order.
 *
 * It's all based on the one ArUco marker, so it makes setting everything up kinda hard.
 */
public class DoorSceneNodeDefinitions
{
   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_ARUCO_MARKER_WIDTH = RigidBodySceneObjectDefinitions.LARGE_MARKER_WIDTH;

   public static final int PULL_DOOR_MARKER_ID = 0;
   public static final int PUSH_DOOR_MARKER_ID = 1;

   // PUSH DOOR

   public static final RigidBodyTransform PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM  = new RigidBodyTransform();
   static
   {
      PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM.getTranslation().setZ(DoorModelParameters.PUSH_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Z);
      PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM.getTranslation().setY(-DoorModelParameters.PUSH_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Y);
   }
   public static final RigidBodyTransform PUSH_DOOR_MARKER_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform leverToPanelTransform = new RigidBodyTransform();
      leverToPanelTransform.getTranslation().setX(DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      leverToPanelTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_LEVER_HANDLE_INSET);
      leverToPanelTransform.getTranslation().setZ(DoorModelParameters.DOOR_LEVER_HANDLE_FROM_BOTTOM_OF_PANEL);

      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame leverFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(panelFrame, leverToPanelTransform);
      ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(leverFrame,
                                                                                                            PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM);
      FramePose3D markerPose = new FramePose3D(markerFrame);
      markerPose.changeFrame(panelFrame);
      markerPose.get(PUSH_DOOR_MARKER_TO_PANEL_TRANSFORM);
   }
   public static final RigidBodyTransform PUSH_DOOR_MARKER_TO_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform panelToFrameTransform = new RigidBodyTransform();
      panelToFrameTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET);
      panelToFrameTransform.getTranslation().setZ(DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);

      ReferenceFrame frameFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameFrame, panelToFrameTransform);
      ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(panelFrame,
                                                                                                            PUSH_DOOR_MARKER_TO_PANEL_TRANSFORM);
      FramePose3D markerPose = new FramePose3D(markerFrame);
      markerPose.changeFrame(frameFrame);
      markerPose.get(PUSH_DOOR_MARKER_TO_FRAME_TRANSFORM);
   }

   // WRONG STUFF

   public static final RigidBodyTransform PULL_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getRotation(), 180.0, 0.0, 0.0);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setX(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET
                                                                + DoorModelParameters.PULL_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Y);
      PULL_DOOR_PANEL_TRANSFORM_TO_MARKER.getTranslation().setZ(DoorModelParameters.PULL_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Z);
   }
   /** The frame has the same origin as the panel. */
   public static final RigidBodyTransform PULL_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER);

   public static final double DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN = 3.0;

   public static final RigidBodyTransform PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getTranslation().add(0.03, 0.09, -0.105);
      EuclidCoreMissingTools.setYawPitchRollDegrees(PULL_DOOR_LEVER_HANDLE_TRANSFORM_TO_MARKER.getRotation(), 90.0, 0.0, 90.0);
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
   static
   {
      PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendRollRotation(Math.PI);
      PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
   }

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
                                     PUSH_DOOR_MARKER_TO_PANEL_TRANSFORM,
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
      return new StaticArUcoRelativeDetectableSceneNode("PushDoorFrame",
                                                        PUSH_DOOR_MARKER_ID,
                                                        DOOR_ARUCO_MARKER_WIDTH,
                                                        PUSH_DOOR_MARKER_TO_FRAME_TRANSFORM,
                                                        DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                        PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                        DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN);
   }

   public static ArUcoDetectableNode createPushDoorLeverHandle()
   {
      return new ArUcoDetectableNode("PushDoorLeverHandle",
                                     PUSH_DOOR_MARKER_ID,
                                     DOOR_ARUCO_MARKER_WIDTH,
                                     PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM,
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
