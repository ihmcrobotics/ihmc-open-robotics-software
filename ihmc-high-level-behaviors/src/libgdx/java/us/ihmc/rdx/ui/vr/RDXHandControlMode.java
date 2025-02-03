package us.ihmc.rdx.ui.vr;

public enum RDXHandControlMode
{
   /* No hand is present on this side: a VR hand action does nothing */
   NONE,
   /* Gripper is present on this side: a VR hand action toggles the gripper state */
   GRIPPER,
   /* The arm is load bearing on this side: a VR hand action loads/unloads the arm */
   LOAD_BEARING
}
