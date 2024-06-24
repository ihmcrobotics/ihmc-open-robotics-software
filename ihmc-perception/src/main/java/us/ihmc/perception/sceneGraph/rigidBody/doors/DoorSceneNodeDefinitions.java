package us.ihmc.perception.sceneGraph.rigidBody.doors;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * The DoorSceneNodeDefinitions class provides definitions and methods to create and manage
 * scene graph nodes representing various components of a door, such as panels, frames, knobs,
 * and handles. It is used to ensure that the necessary scene graph nodes are added to the
 * scene graph in the correct order and with the correct transformations.
 *
 * This class is designed to work with a scene graph and an ArUco marker-based door model.
 * It defines constants for the names of various door components and their corresponding
 * transformations, visual models, and marker sizes.
 */

public class DoorSceneNodeDefinitions
{
   // RIGHT PANEL AND OPENERS
   public static final String RIGHT_DOOR_PANEL_NAME = "RightDoorPanel";
   public static final String RIGHT_DOOR_LEVER_HANDLE_NAME = "RightDoorLeverHandle";
   public static final String RIGHT_DOOR_KNOB_NAME = "RightDoorKnob";
   public static final String RIGHT_DOOR_EMERGENCY_BAR_NAME = "RightDoorEmergencyBar";
   public static final RigidBodyTransform RIGHT_DOOR_MARKER_TO_OPENER_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform RIGHT_DOOR_OPENER_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      RIGHT_DOOR_MARKER_TO_OPENER_TRANSFORM.getTranslation().setZ(DoorModelParameters.RIGHT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Z);
      RIGHT_DOOR_MARKER_TO_OPENER_TRANSFORM.getTranslation().setY(-DoorModelParameters.RIGHT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Y);
      RIGHT_DOOR_OPENER_TO_MARKER_TRANSFORM.setAndInvert(RIGHT_DOOR_MARKER_TO_OPENER_TRANSFORM);
   }
   public static final RigidBodyTransform RIGHT_DOOR_MARKER_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform RIGHT_DOOR_PANEL_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform leverToPanelTransform = new RigidBodyTransform();
      leverToPanelTransform.getTranslation().setX(DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      leverToPanelTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_OPENER_INSET);
      leverToPanelTransform.getTranslation().setZ(DoorModelParameters.DOOR_OPENER_FROM_BOTTOM_OF_PANEL);

      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame leverFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(panelFrame, leverToPanelTransform);
      ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(leverFrame, RIGHT_DOOR_MARKER_TO_OPENER_TRANSFORM);
      FramePose3D markerPose = new FramePose3D(markerFrame);
      markerPose.changeFrame(panelFrame);
      markerPose.get(RIGHT_DOOR_MARKER_TO_PANEL_TRANSFORM);
      RIGHT_DOOR_PANEL_TO_MARKER_TRANSFORM.setAndInvert(RIGHT_DOOR_MARKER_TO_PANEL_TRANSFORM);
   }

   // PUSH FRAME
   public static final String PUSH_DOOR_FRAME_NAME = "PushDoorFrame";
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

   //LEFT DOOR AND OPENERS
   public static final String LEFT_DOOR_PANEL_NAME = "LeftDoorPanel";
   public static final String LEFT_DOOR_LEVER_HANDLE_NAME = "LeftDoorLeverHandle";
   public static final String LEFT_DOOR_KNOB_NAME = "LeftDoorKnob";
   public static final String LEFT_DOOR_EMERGENCY_BAR_NAME = "LeftDoorEmergencyBar";
   public static final RigidBodyTransform LEFT_DOOR_MARKER_TO_OPENER_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform LEFT_DOOR_OPENER_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      LEFT_DOOR_MARKER_TO_OPENER_TRANSFORM.getTranslation().setZ(DoorModelParameters.LEFT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Z);
      LEFT_DOOR_MARKER_TO_OPENER_TRANSFORM.getTranslation().setY(-DoorModelParameters.LEFT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Y);
      LEFT_DOOR_OPENER_TO_MARKER_TRANSFORM.setAndInvert(LEFT_DOOR_MARKER_TO_OPENER_TRANSFORM);
   }
   public static final RigidBodyTransform LEFT_DOOR_MARKER_TO_PANEL_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform LEFT_DOOR_PANEL_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      RigidBodyTransform leverToPanelTransform = new RigidBodyTransform();
      leverToPanelTransform.getRotation().setToYawOrientation(Math.PI);
      leverToPanelTransform.getTranslation().setX(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0);
      leverToPanelTransform.getTranslation().setY(DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_OPENER_INSET);
      leverToPanelTransform.getTranslation().setZ(DoorModelParameters.DOOR_OPENER_FROM_BOTTOM_OF_PANEL);

      ReferenceFrame panelFrame = ReferenceFrameMissingTools.constructARootFrame();
      ReferenceFrame leverFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(panelFrame, leverToPanelTransform);
      ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(leverFrame, LEFT_DOOR_MARKER_TO_OPENER_TRANSFORM);
      FramePose3D markerPose = new FramePose3D(markerFrame);
      markerPose.changeFrame(panelFrame);
      markerPose.get(LEFT_DOOR_MARKER_TO_PANEL_TRANSFORM);
      LEFT_DOOR_PANEL_TO_MARKER_TRANSFORM.setAndInvert(LEFT_DOOR_MARKER_TO_PANEL_TRANSFORM);
   }

   // PULL FRAME
   public static final String PULL_DOOR_FRAME_NAME = "PullDoorFrame";
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
   public static final RigidBodyTransform LEFT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      LEFT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
   }
   public static final RigidBodyTransform RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendRollRotation(Math.PI);
      RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
   }
   public static final String DOOR_KNOB_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorKnob/DoorKnob.g3dj";
   public static final RigidBodyTransform DOOR_KNOB_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorEmergencyBar/DoorEmergencyBar.g3dj";
   public static final RigidBodyTransform LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendRollRotation(Math.PI);
      LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendTranslation(0.0, 0.25, 0.0);
   }
   public static final RigidBodyTransform RIGHT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   public static final String DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorPullHandle/pullhandle.g3dj";
   public static final RigidBodyTransform DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendRollRotation(Math.PI);
      DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendYawRotation(Math.PI);
      DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.appendTranslation(0, 0, 0.05);
   }
   public static final TIntDoubleMap ARUCO_MARKER_SIZES = new TIntDoubleHashMap();
   static
   {
      ARUCO_MARKER_SIZES.put(DoorModelParameters.RIGHT_PUSH_DOOR_MARKER_ID, SceneObjectDefinitions.LARGE_MARKER_WIDTH);
      ARUCO_MARKER_SIZES.put(DoorModelParameters.LEFT_PULL_DOOR_MARKER_ID, SceneObjectDefinitions.LARGE_MARKER_WIDTH);
   }

   public static final double DOOR_YOLO_STATIC_MAXIMUM_DISTANCE_TO_LOCK_IN = 1.5;
   public static final RigidBodyTransform DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM.appendRollRotation(Math.PI);
      DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM.appendYawRotation(Math.PI);
   }

   public static void ensureNodesAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      ArUcoMarkerNode leftPushDoorArUcoMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(DoorModelParameters.LEFT_PUSH_DOOR_MARKER_ID);
      if (leftPushDoorArUcoMarker != null)
      {
         ensureDoorNodeTreeExists(sceneGraph, modificationQueue, leftPushDoorArUcoMarker, RobotSide.LEFT, true);
      }

      ArUcoMarkerNode leftPullDoorArUcoMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(DoorModelParameters.LEFT_PULL_DOOR_MARKER_ID);
      if (leftPullDoorArUcoMarker != null)
      {
         ensureDoorNodeTreeExists(sceneGraph, modificationQueue, leftPullDoorArUcoMarker, RobotSide.LEFT, false);
      }

      ArUcoMarkerNode rightPushDoorArUcoMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(DoorModelParameters.RIGHT_PUSH_DOOR_MARKER_ID);
      if (rightPushDoorArUcoMarker != null)
      {
         ensureDoorNodeTreeExists(sceneGraph, modificationQueue, rightPushDoorArUcoMarker, RobotSide.RIGHT, true);
      }

      ArUcoMarkerNode rightPullDoorArUcoMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(DoorModelParameters.RIGHT_PULL_DOOR_MARKER_ID);
      if (rightPullDoorArUcoMarker != null)
      {
         ensureDoorNodeTreeExists(sceneGraph, modificationQueue, rightPullDoorArUcoMarker, RobotSide.RIGHT, false);
      }
   }

   public static void ensureDoorNodeTreeExists(SceneGraph sceneGraph,
                                               SceneGraphModificationQueue modificationQueue,
                                               SceneNode parentNode,
                                               RobotSide sideDoorOpensFrom,
                                               boolean pushDoor)
   {
      String doorPanelNodeName = sideDoorOpensFrom == RobotSide.LEFT ? LEFT_DOOR_PANEL_NAME : RIGHT_DOOR_PANEL_NAME;
      SceneNode doorPanelNode = sceneGraph.getNamesToNodesMap().get(doorPanelNodeName);
      if (doorPanelNode == null)
      {
         doorPanelNode = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                          doorPanelNodeName,
                                                          sceneGraph.getIDToNodeMap(),
                                                          parentNode.getID(),
                                                          sideDoorOpensFrom == RobotSide.LEFT ?
                                                                LEFT_DOOR_PANEL_TO_MARKER_TRANSFORM :
                                                                RIGHT_DOOR_PANEL_TO_MARKER_TRANSFORM,
                                                          DOOR_PANEL_VISUAL_MODEL_FILE_PATH,
                                                          pushDoor ?
                                                                PUSH_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM :
                                                                PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                          sceneGraph.getCRDTInfo());
         modificationQueue.accept(new SceneGraphNodeAddition(doorPanelNode, parentNode));
      }

      String doorFrameNodeName = pushDoor ? PUSH_DOOR_FRAME_NAME : PULL_DOOR_FRAME_NAME;
      SceneNode doorFrameNode = sceneGraph.getNamesToNodesMap().get(doorFrameNodeName);
      if (doorFrameNode == null)
      {
         doorFrameNode = new StaticRelativeSceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                     doorFrameNodeName,
                                                     sceneGraph.getIDToNodeMap(),
                                                     doorPanelNode.getID(),
                                                     pushDoor ? PUSH_DOOR_FRAME_TO_PANEL_TRANSFORM : PULL_DOOR_FRAME_TO_PANEL_TRANSFORM,
                                                     DOOR_FRAME_VISUAL_MODEL_FILE_PATH,
                                                     pushDoor ?
                                                           PUSH_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM :
                                                           PULL_DOOR_FRAME_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                     DOOR_FRAME_MAXIMUM_DISTANCE_TO_LOCK_IN,
                                                     sceneGraph.getCRDTInfo());
         modificationQueue.accept(new SceneGraphNodeAddition(doorFrameNode, parentNode));
      }

      RigidBodyTransform doorOpenerToMarkerTransform =
            sideDoorOpensFrom == RobotSide.LEFT ? LEFT_DOOR_OPENER_TO_MARKER_TRANSFORM : RIGHT_DOOR_OPENER_TO_MARKER_TRANSFORM;

      // Add door lever
      {
         String doorLeverNodeName = sideDoorOpensFrom == RobotSide.LEFT ? LEFT_DOOR_LEVER_HANDLE_NAME : RIGHT_DOOR_LEVER_HANDLE_NAME;
         SceneNode doorLeverNode = sceneGraph.getNamesToNodesMap().get(doorLeverNodeName);
         if (doorLeverNode == null)
         {
            RigidBodyTransform leverToVisualModelTransform = sideDoorOpensFrom == RobotSide.LEFT ?
                  LEFT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM :
                  RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
            doorLeverNode = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                             doorLeverNodeName,
                                                             sceneGraph.getIDToNodeMap(),
                                                             parentNode.getID(),
                                                             doorOpenerToMarkerTransform,
                                                             DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                                             leverToVisualModelTransform,
                                                             sceneGraph.getCRDTInfo());
            modificationQueue.accept(new SceneGraphNodeAddition(doorLeverNode, parentNode));
         }
      }

      // Add door knob
      {
         String doorKnobNodeName = sideDoorOpensFrom == RobotSide.LEFT ? LEFT_DOOR_KNOB_NAME : RIGHT_DOOR_KNOB_NAME;
         SceneNode doorKnobNode = sceneGraph.getNamesToNodesMap().get(doorKnobNodeName);
         if (doorKnobNode == null)
         {
            doorKnobNode = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                            doorKnobNodeName,
                                                            sceneGraph.getIDToNodeMap(),
                                                            parentNode.getID(),
                                                            doorOpenerToMarkerTransform,
                                                            DOOR_KNOB_VISUAL_MODEL_FILE_PATH,
                                                            DOOR_KNOB_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                            sceneGraph.getCRDTInfo());
            modificationQueue.accept(new SceneGraphNodeAddition(doorKnobNode, parentNode));
         }
      }

      // Add door push bar
      {
         String doorPushBarNodeName = sideDoorOpensFrom == RobotSide.LEFT ? LEFT_DOOR_EMERGENCY_BAR_NAME : RIGHT_DOOR_EMERGENCY_BAR_NAME;
         SceneNode doorPushBarNode = sceneGraph.getNamesToNodesMap().get(doorPushBarNodeName);
         if (doorPushBarNode == null)
         {
            doorPushBarNode = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                               doorPushBarNodeName,
                                                               sceneGraph.getIDToNodeMap(),
                                                               parentNode.getID(),
                                                               doorOpenerToMarkerTransform,
                                                               DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH,
                                                               LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM,
                                                               sceneGraph.getCRDTInfo());
            modificationQueue.accept(new SceneGraphNodeAddition(doorPushBarNode, parentNode));
         }
      }
   }
}