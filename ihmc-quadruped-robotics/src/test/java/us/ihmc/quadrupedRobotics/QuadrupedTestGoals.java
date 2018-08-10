package us.ihmc.quadrupedRobotics;

import us.ihmc.robotics.testing.YoVariableTestGoal;

public class QuadrupedTestGoals
{
   public static YoVariableTestGoal notFallen(QuadrupedTestYoVariables variables)
   {
      return YoVariableTestGoal.deltaGreaterThan(variables.getRobotBodyZ(), variables.getGroundPlanePointZ(), 0.0);
   }

   public static YoVariableTestGoal bodyHeight(QuadrupedTestYoVariables variables, double height)
   {
      return YoVariableTestGoal.deltaGreaterThan(variables.getRobotBodyZ(), variables.getGroundPlanePointZ(), height);
   }
   
   public static YoVariableTestGoal timeInFuture(QuadrupedTestYoVariables variables, double durationFromNow)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + durationFromNow);
   }
}
