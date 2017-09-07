package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

public interface PathTypeStepParameters
{
   // The angle between the path direction and the direction the robot is facing.
   // E.g. walking forward, 0.0
   // walking to the left, the robot is turned -PI/2 radians relative to the direction of motion.
   public double getAngle();

   // The nominal lateral distance between feet.
   // For forward walking this is the lateral distance between feet.
   // For side stepping this is the minimum distance between feet.
   public double getStepWidth();

   // Used to find periodic displacement along path direction
   // This is the step displacement.
   // For side stepping the largest distance between feet will be the stepWidth + stepLength
   public double getStepLength();
   
   public double getTurningOpenStepAngle();//radians
   public double getTurningCloseStepAngle();//radians
   public double getTurningStepWidth();
}
