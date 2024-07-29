package us.ihmc.rdx.ui.remoteCaptury;

public enum RDXCapturyMode
{
   INPUTS_DISABLED("Inputs disabled"),
   WHOLE_BODY_IK_STREAMING("Whole body IK streaming");

   private String readableName;

   RDXCapturyMode(String readableName)
   {
      this.readableName = readableName;
   }

   public String getReadableName()
   {
      return readableName;
   }
}
