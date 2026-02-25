package us.ihmc.behaviors.simulation.door;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

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
   public static final String DOOR_PULL_HANDLE_NAME = "DoorPullHandle";
   // RIGHT PANEL AND OPENERS
   public static final String RIGHT_DOOR_PANEL_NAME = "RightDoorPanel";
   public static final String RIGHT_DOOR_LEVER_HANDLE_NAME = "RightDoorLeverHandle";
   public static final String RIGHT_DOOR_KNOB_NAME = "RightDoorKnob";
   public static final String RIGHT_DOOR_EMERGENCY_BAR_NAME = "RightDoorEmergencyBar";

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
   public static final String DOOR_PANEL_VISUAL_MODEL_FILE_PATH = "environmentObjects/doorPanel/doorPanel.g3dj";
   public static final RigidBodyTransform PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   //   static
   //   {
   //      PULL_DOOR_PANEL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.45);
   //   }
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