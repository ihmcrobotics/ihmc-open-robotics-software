package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * We want to measure this stuff in the right order.
 *
 * It's all based on the one ArUco marker, so it makes setting everything up kinda hard.
 */
public class DoorSceneNodeDefinitions
{

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
   public static final RigidBodyTransform PUSH_DOOR_FRAME_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform panelToFrameTransform = new RigidBodyTransform();
      panelToFrameTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET);
      panelToFrameTransform.getTranslation().setZ(DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);

      ReferenceFrame frameFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameFrame, panelToFrameTransform);

      FramePose3D framePose = new FramePose3D(frameFrame);
      framePose.changeFrame(panelFrame);
      framePose.get(PUSH_DOOR_FRAME_TO_PANEL_TRANSFORM);
   }

   // PULL DOOR

   public static final RigidBodyTransform PULL_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM  = new RigidBodyTransform();
   static
   {
      PULL_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM.getTranslation().setZ(DoorModelParameters.PULL_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Z);
      PULL_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM.getTranslation().setY(-DoorModelParameters.PULL_SIDE_ARUCO_MARKER_TO_LEVER_AXIS_Y);
   }
   public static final RigidBodyTransform PULL_DOOR_MARKER_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform leverToPanelTransform = new RigidBodyTransform();
      leverToPanelTransform.getRotation().setToYawOrientation(Math.PI);
      leverToPanelTransform.getTranslation().setX(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      leverToPanelTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_LEVER_HANDLE_INSET);
      leverToPanelTransform.getTranslation().setZ(DoorModelParameters.DOOR_LEVER_HANDLE_FROM_BOTTOM_OF_PANEL);

      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame leverFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(panelFrame, leverToPanelTransform);
      ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(leverFrame,
                                                                                                            PULL_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM);
      FramePose3D markerPose = new FramePose3D(markerFrame);
      markerPose.changeFrame(panelFrame);
      markerPose.get(PULL_DOOR_MARKER_TO_PANEL_TRANSFORM);
   }
   public static final RigidBodyTransform PULL_DOOR_FRAME_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform panelToFrameTransform = new RigidBodyTransform();
      panelToFrameTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_HINGE_OFFSET);
      panelToFrameTransform.getTranslation().setZ(DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);

      ReferenceFrame frameFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameFrame, panelToFrameTransform);

      FramePose3D framePose = new FramePose3D(frameFrame);
      framePose.changeFrame(panelFrame);
      framePose.get(PULL_DOOR_FRAME_TO_PANEL_TRANSFORM);
   }

   public static final double DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN = 2.0;

   // TODO: These transforms need to be verified.
   public static final String DOOR_PANEL_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorPanel/DoorPanel.g3dj";
   public static final RigidBodyTransform PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform PUSH_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_FRAME_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorFrame/DoorFrame.g3dj";
   public static final RigidBodyTransform PULL_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj";
   public static final RigidBodyTransform PULL_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      PULL_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
   }
   public static final RigidBodyTransform PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendRollRotation(Math.PI);
      PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
   }

   public static PredefinedRigidBodySceneNode createPullDoorPanel(SceneGraph sceneGraph, SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                           "PullDoorPanel",
                                                                           DOOR_PANEL_VISUAL_MODEL_FILE_PATH,
                                                                           PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(PULL_DOOR_MARKER_TO_PANEL_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
      return node;
   }

   public static PredefinedRigidBodySceneNode createPushDoorPanel(SceneGraph sceneGraph, SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                           "PushDoorPanel",
                                                                           DOOR_PANEL_VISUAL_MODEL_FILE_PATH,
                                                                           PUSH_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(PUSH_DOOR_MARKER_TO_PANEL_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
      return node;
   }

   public static void createPullDoorFrame(SceneGraph sceneGraph, SceneNode parentNode)
   {
      StaticRelativeSceneNode node = new StaticRelativeSceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                 "PullDoorFrame",
                                                                 DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                                 PULL_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                                 DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN);
      node.getNodeToParentFrameTransform().set(PULL_DOOR_FRAME_TO_PANEL_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
   }

   public static void createPushDoorFrame(SceneGraph sceneGraph, SceneNode parentNode)
   {
      StaticRelativeSceneNode node = new StaticRelativeSceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                 "PushDoorFrame",
                                                                 DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                                 PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                                 DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN);
      node.getNodeToParentFrameTransform().set(PUSH_DOOR_FRAME_TO_PANEL_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
   }

   public static void createPushDoorLeverHandle(SceneGraph sceneGraph, SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                           "PushDoorLeverHandle",
                                                                           DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                                                           PUSH_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(PUSH_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
   }

   public static void createPullDoorLeverHandle(SceneGraph sceneGraph, SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                           "PullDoorLeverHandle",
                                                                           DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                                                           PULL_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(PULL_DOOR_MARKER_TO_LEVER_HANDLE_TRANSFORM);
      node.setOriginalParentID(sceneGraph, parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      parentNode.getChildren().add(node);
   }
}
