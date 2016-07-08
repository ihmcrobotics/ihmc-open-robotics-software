package us.ihmc.quadrupedRobotics;

import us.ihmc.robotics.testing.YoVariableTestGoal;

public class QuadrupedTestGoals
{
   public static YoVariableTestGoal notFallen(QuadrupedTestYoVariables variables)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), variables.getStanceHeight().getDoubleValue() / 2.0);
   }
}
