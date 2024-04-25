package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;

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
}