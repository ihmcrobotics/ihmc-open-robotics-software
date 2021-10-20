package us.ihmc.gdx.vr;

/**
 * Button ids on VR controllers
 */
public class GDXVRControllerButtons
{
   public static final int System = 0;
   public static final int ApplicationMenu = 1;
   public static final int Grip = 2;
   public static final int DPad_Left = 3;
   public static final int DPad_Up = 4;
   public static final int DPad_Right = 5;
   public static final int DPad_Down = 6;
   public static final int A = 7;

   public static final int ProximitySensor = 31;

   public static final int Axis0 = 32;
   public static final int Axis1 = 33;
   public static final int Axis2 = 34;
   public static final int Axis3 = 35;
   public static final int Axis4 = 36;

   // aliases for well known controllers
   public static final int SteamVR_Touchpad = Axis0;
   public static final int SteamVR_Trigger = Axis1;

   public static final int Dashboard_Back = Grip;

   public static final int INDEX_B = 1;
   public static final int INDEX_A = 2;
   public static final int INDEX_JOYSTICK_BUTTON = 32;
   public static final int INDEX_TRIGGER_AS_BUTTON = 33;
   /**
    * Index controllers:
    * Touchpad and joystick:
    *        1.0
    * x -1.0     1.0
    *       -1.0
    *        y
    * Trigger:
    * 0.0 unpressed to 1.0 pressed before button is clicked.
    *
    */
   public static final int INDEX_JOYSTICK_AND_TOUCHPAD_R_AXIS = 0;
   public static final int INDEX_TRIGGER_R_AXIS = 0;

}
