package us.ihmc.robotics.controllers.pidGains;

/**
 * This enum is used in the context of 3D PID gains and can be set to determine if
 * multiple axes should be controlled using the same gain. E.g. if orientation gains
 * are created with {@link GainCoupling#XY} the YoVariables to tune the X and Y
 * direction gains are not duplicated, while the Z gains will have individual tuning
 * variables.
 */
public enum GainCoupling
{
   XY,
   YZ,
   XZ,
   XYZ,
   NONE;
}
