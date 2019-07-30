package us.ihmc.quadrupedRobotics;

import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.robotics.testing.YoVariableTestGoal.StringProvider;

public class QuadrupedTestGoals
{
   public static YoVariableTestGoal notFallen(QuadrupedTestYoVariables variables)
   {
      YoVariableTestGoal minHeightGoal = YoVariableTestGoal.deltaGreaterThan(variables.getRobotBodyZ(), variables.getGroundPlanePointZ(), 0.0);
      YoVariableTestGoal fallenFlag = YoVariableTestGoal.booleanEquals(variables.getIsFallDetected(), false);
      StringProvider stringProvider = () -> "Fell from reason " + variables.getFallDetectionType() + "\n.";
      return YoVariableTestGoal.and(stringProvider, minHeightGoal, fallenFlag);
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
