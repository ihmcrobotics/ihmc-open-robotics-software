package us.ihmc.rdx.ui.vr;

public enum VRMenuGuideMode
{
   OFF, IDLE,
   PRESS_LEFT_A, PRESS_LEFT_B, PUSH_LEFT_JOYSTICK, PULL_LEFT_JOYSTICK,
   PRESS_RIGHT_A, PRESS_RIGHT_B, PUSH_RIGHT_JOYSTICK, PULL_RIGHT_JOYSTICK,
   MOVE_LEFT, MOVE_RIGHT, MOVE_LEFT_RIGHT;

   public boolean equals(VRMenuGuideMode other) {
      return this.name().equals(other.name()) && this.ordinal() == other.ordinal();
   }
}
