package us.ihmc.perception.objects;

public class DoorModelParameters
{
   /* These measurements from the simulation door model, measured in Blender. */
   /** The thickness of the door panel. */
   public static final double DOOR_PANEL_THICKNESS = 0.0508;
   /** The vertical length of the panel. */
   public static final double DOOR_PANEL_HEIGHT = 2.0447;
   /** The horizontal length of the panel. */
   public static final double DOOR_PANEL_WIDTH = 0.9144;
   /** Distance the handle joint in from the edge of the panel. */
   public static final double DOOR_LEVER_HANDLE_INSET = 0.05;
   /** We place the lever handle a little below halfway up the door like it normally is on doors. */
   public static final double DOOR_LEVER_HANDLE_DISTANCE_BELOW_MID_HEIGHT = 0.13;
   /** Mount the panel up off the ground a little so it's not dragging. */
   public static final double DOOR_PANEL_GROUND_GAP_HEIGHT = 0.02;
   /** Place the panel away from the hinge a little.
    *  TODO: Rethink this, it's not very realistic. */
   public static final double DOOR_PANEL_HINGE_OFFSET = 0.005;
   /** Distance from the frame post to the frame model's origin. */
   public static final double DOOR_FRAME_HINGE_OFFSET = 0.006;

   public static final double ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Z_IN_PANEL_FRAME = 1.03981;
   public static final double ARUCO_MARKER_PUSH_SIDE_BOTTOM_RIGHT_CORNER_Y_IN_PANEL_FRAME = 0.780302;

   public static final double ARUCO_MARKER_PULL_SIDE_BOTTOM_RIGHT_CORNER_Z_IN_PANEL_FRAME = 1.03981;
   public static final double ARUCO_MARKER_PULL_SIDE_BOTTOM_RIGHT_CORNER_Y_IN_PANEL_FRAME = 0.780301;
}
