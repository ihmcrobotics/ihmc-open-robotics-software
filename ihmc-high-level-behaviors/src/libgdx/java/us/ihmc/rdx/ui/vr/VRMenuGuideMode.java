package us.ihmc.rdx.ui.vr;

public enum VRMenuGuideMode
{
   OFF, IDLE, ACTIVATE, VALIDATE, DIRECT_WITH_JOYSTICK, MOVE;

   public boolean equals(VRMenuGuideMode other) {
      return this.name().equals(other.name()) && this.ordinal() == other.ordinal();
   }
}
