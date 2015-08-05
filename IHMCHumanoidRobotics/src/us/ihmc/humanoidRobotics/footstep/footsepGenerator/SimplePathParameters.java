package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

public class SimplePathParameters implements PathTypeStepParameters
{
   double stepLength, stepWidth, orientationRelativeToPathDirection, turningOpenAngle, turningCloseAngle, turningStepWidth;

   public SimplePathParameters(double stepLength, double stepWidth, double orientationRelativeToPathDirection, double turningOpenAngle,
                               double turningCloseAngle, double turningStepWidth)
   {
      this.stepLength = stepLength;
      this.stepWidth = stepWidth;
      this.orientationRelativeToPathDirection = orientationRelativeToPathDirection;
      this.turningOpenAngle = turningOpenAngle;
      this.turningCloseAngle = turningCloseAngle;
      this.turningStepWidth = turningStepWidth;
   }

   public double getAngle()
   {
      return orientationRelativeToPathDirection;
   }

   public double getStepWidth()
   {
      return stepWidth;
   }

   public double getStepLength()
   {
      return stepLength;
   }

   public double getTurningOpenStepAngle()
   {
      return turningOpenAngle;
   }

   public double getTurningCloseStepAngle()
   {
      return turningCloseAngle;
   }

   public double getTurningStepWidth()
   {
      return turningStepWidth;
   }


   public void setAngle(double orientationRelativeToPathDirection)
   {
      this.orientationRelativeToPathDirection  = orientationRelativeToPathDirection;
   }

   public void setStepWidth(double stepWidth)
   {
      this.stepWidth = stepWidth;

   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;

   }

   public void setTurningOpenStepAngle(double turningOpenAngle)
   {
      this.turningOpenAngle = turningOpenAngle;

   }

   public void setTurningCloseStepAngle(double turningCloseAngle)
   {
      this.turningCloseAngle = turningCloseAngle;

   }

   public void setTurningStepWidth(double turningStepWidth)
   {
      this.turningStepWidth = turningStepWidth;
   }

}
