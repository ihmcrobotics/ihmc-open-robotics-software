package us.ihmc.behaviors.simulation.door;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * The parameters for the real and simulated door.
 * We are trying to keep the simulation door representing the
 * real door pretty much exactly for now.
 *
 * Remeasured by dcalvert on 7/11/23:
 * Push door (real):
 * Thickness - 3.4 cm
 * Lever axis inset - 6.2 cm
 * Lever axis height - 91.5 cm
 * 91.4 cm panel width
 * 203.3 cm panel height
 * 104.4 marker z from bottom of panel
 * 12.7 cm lever below marker
 * 8.85 cm lever right of marker
 * 5 cm lever away from panel
 * 9 cm lever length
 */
public class DoorModelParameters
{
   /* These measurements from the simulation door model, measured in Blender. */
   /** The thickness of the door panel. */
   public static final double DOOR_PANEL_THICKNESS = 0.034;
   /** The vertical length of the panel. */
   public static final double DOOR_PANEL_HEIGHT = 2.033;
   /** The horizontal length of the panel. */
   public static final double DOOR_PANEL_WIDTH = 0.924;
   /** Distance the handle joint in from the edge of the panel. */
   public static final double DOOR_OPENER_INSET = 0.062;
   /** We place the lever handle up from the bottom of the panel as measured on our lab door. */
   public static final double DOOR_OPENER_FROM_BOTTOM_OF_PANEL = 0.915;
   /** Mount the panel up off the ground a little so it's not dragging. */
   public static final double DOOR_PANEL_GROUND_GAP_HEIGHT = 0.02;
   /** Place the panel away from the hinge a little. */
   public static final double DOOR_PANEL_HINGE_OFFSET = 0.002;
   /** Distance from the frame post to the frame model's origin. */
   public static final double DOOR_FRAME_HINGE_OFFSET = 0.006;
   /** Frame post X size. */
   public static final double DOOR_FRAME_PILLAR_SIZE_X = 0.0889;
   /** Frame post Z size. */
   public static final double DOOR_FRAME_PILLAR_SIZE_Z = 2.159;
   /** The angle of the lever in which the bolt is fully drawn i.e. the end stop */
   public static final double DOOR_LEVER_MAX_TURN_ANGLE = 0.4 * Math.PI / 2.0;
   /**
    * The torque required to turn the lever to the max angle.
    * 4 Nm seems to be typical door handle torque, but we're making it easy.
    */
   public static final double DOOR_LEVER_MAX_TORQUE = 1.0;
   public static final double DOOR_BOLT_HEIGHT = 0.015;
   public static final double DOOR_BOLT_HOLE_HEIGHT = DOOR_BOLT_HEIGHT + 0.01;
   public static final double DOOR_BOLT_TRAVEL = 0.015;

   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_ARUCO_MARKER_WIDTH = SceneObjectDefinitions.LARGE_MARKER_WIDTH;
   public static final int LEFT_PULL_DOOR_MARKER_ID = 0;
   public static final int RIGHT_PUSH_DOOR_MARKER_ID = 1;
   public static final int RIGHT_PULL_DOOR_MARKER_ID = 10;
   public static final int LEFT_PUSH_DOOR_MARKER_ID = 11;

   /**
    * It is actually important to measure the ArUco marker pose relative to the lever handle,
    * as that's what's most important to get right.
    */
   public static final double RIGHT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Z = 0.127;
   public static final double RIGHT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Y = 0.0885;

   public static final double LEFT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Z = 0.127;
   public static final double LEFT_SIDE_ARUCO_MARKER_TO_OPENER_AXIS_Y = -0.0885 - DOOR_ARUCO_MARKER_WIDTH;

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
   static
   {
      PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.45);
   }
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
   public static final RigidBodyTransform DOOR_KNOB_TO_YOLO_VISUAL_MODEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      DOOR_KNOB_TO_YOLO_VISUAL_MODEL_TRANSFORM.appendRollRotation(Math.PI);
   }
   public static final RigidBodyTransform DOOR_PUSH_BAR_TO_YOLO_VISUAL_MODEL_TRANSFORM = new RigidBodyTransform();
   static
   {
      DOOR_PUSH_BAR_TO_YOLO_VISUAL_MODEL_TRANSFORM.appendRollRotation(Math.PI);
      DOOR_PUSH_BAR_TO_YOLO_VISUAL_MODEL_TRANSFORM.appendTranslation(0.0, 0.26, 0.0);
   }
}