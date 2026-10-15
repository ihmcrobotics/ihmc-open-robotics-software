package us.ihmc.rdx.simulation.environment.object;

/**
 * Dimensions and visual paths for the lab door.
 * <p>
 * Remeasured by dcalvert on 7/11/23. Push door (real): thickness 3.4 cm, lever axis inset 6.2 cm,
 * lever axis height 91.5 cm, panel 91.4 cm by 203.3 cm, lever 5 cm off the panel and 9 cm long.
 * Panel width, height, and thickness below are the simulation model, measured in Blender.
 */
public class DoorModelParameters
{
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

   public static final String DOOR_PANEL_VISUAL_MODEL_FILE_PATH = "environmentObjects/doorPanel/doorPanel.g3dj";
   public static final String DOOR_FRAME_VISUAL_MODEL_FILE_PATH = "environmentObjects/door/doorFrame/DoorFrame.g3dj";
   public static final String DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH = "environmentObjects/door_handle/door_handle.glb";
}
