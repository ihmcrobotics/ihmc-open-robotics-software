package us.ihmc.rdx.ui.vr;

public enum RDXVRMode
{
   INPUTS_DISABLED("Inputs Disabled"),
   WHOLE_BODY_IK_STREAMING("Whole Body Kinematic Streaming"),
   FOOTSTEP_PLACEMENT("Manual Footstep Placement"),
   FOOTSTEP_STREAMING("Footstep Streaming"),
   JOYSTICK_WALKING("Joystick Walking");

   private String readableName;

   RDXVRMode(String readableName)
   {
      this.readableName = readableName;
   }

   public String getReadableName()
   {
      return readableName;
   }
}
