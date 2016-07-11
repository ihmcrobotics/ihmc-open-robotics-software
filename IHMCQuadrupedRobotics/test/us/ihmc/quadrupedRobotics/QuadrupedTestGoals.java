package us.ihmc.quadrupedRobotics;

import us.ihmc.robotics.testing.YoVariableTestGoal;

public class QuadrupedTestGoals
{
   public static YoVariableTestGoal notFallen(QuadrupedForceTestYoVariables variables)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), variables.getStanceHeight().getDoubleValue() / 2.0);
   }
   
   public static YoVariableTestGoal notFallen(QuadrupedPositionTestYoVariables variables)
   {
      return YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), variables.getDesiredCoMPositionZ().getDoubleValue() / 2.0);
   }
}
