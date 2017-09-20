package us.ihmc.quadrupedRobotics;

import us.ihmc.robotics.testing.YoVariableTestGoal;

public class QuadrupedTestGoals
{
   public static YoVariableTestGoal notFallen(QuadrupedForceTestYoVariables variables)
   {
      return YoVariableTestGoal.deltaGreaterThan(variables.getRobotBodyZ(), variables.getGroundPlanePointZ(), variables.getYoComPositionInputZ().getDoubleValue() / 2.0);
   }
   
   public static YoVariableTestGoal notFallen(QuadrupedPositionTestYoVariables variables)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), variables.getDesiredCoMPositionZ().getDoubleValue() / 2.0);
   }
   
   public static YoVariableTestGoal timeInFuture(QuadrupedTestYoVariables variables, double durationFromNow)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + durationFromNow);
   }
}
