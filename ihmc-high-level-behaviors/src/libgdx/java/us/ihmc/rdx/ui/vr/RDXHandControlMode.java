package us.ihmc.rdx.ui.vr;

public enum RDXHandControlMode
{
   /* No hand is present on this side: a VR hand action does nothing */
   NONE,
   /* A hand with fingers or gripper is present on this side: the user fingers control the robot fingers */
   FINGER_STREAMING,
   /* A hand with fingers or gripper is present on this side: the user triggers specific configuration */
   HAND_CONFIGURATION,
   /* The arm is load bearing on this side: a VR hand action loads/unloads the arm */
   LOAD_BEARING
}
