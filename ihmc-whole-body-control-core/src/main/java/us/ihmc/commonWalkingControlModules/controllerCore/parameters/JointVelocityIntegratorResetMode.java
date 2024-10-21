package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

public enum JointVelocityIntegratorResetMode
{
   CURRENT_VELOCITY, ZERO_VELOCITY, REFERENCE_VELOCITY;

   public static final JointVelocityIntegratorResetMode[] values = values();
}
