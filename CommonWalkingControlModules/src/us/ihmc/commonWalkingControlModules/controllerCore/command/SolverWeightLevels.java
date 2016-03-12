package us.ihmc.commonWalkingControlModules.controllerCore.command;

public enum SolverWeightLevels
{
   HIGH, MEDIUM, LOW, HARD_CONSTRAINT;

   public double getWeightValue()
   {
      switch (this)
      {
      case HARD_CONSTRAINT:
         return Double.POSITIVE_INFINITY;
      case HIGH:
         return 10.0;
      case LOW:
         return 0.10;
      case MEDIUM:
      default:
         return 1.00;
      }
   }
}
