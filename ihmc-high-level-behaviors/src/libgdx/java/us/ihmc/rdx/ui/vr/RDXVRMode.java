package us.ihmc.rdx.ui.vr;

public enum RDXVRMode
{
   INPUTS_DISABLED("Inputs disabled"),
   FOOTSTEP_PLACEMENT("Footstep placement"),
   JOYSTICK_WALKING("Joystick walking"),
   WHOLE_BODY_IK_STREAMING("Whole body IK streaming");

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
