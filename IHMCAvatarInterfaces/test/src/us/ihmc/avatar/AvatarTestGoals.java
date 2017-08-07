package us.ihmc.avatar;

import us.ihmc.robotics.testing.YoVariableTestGoal;

public class AvatarTestGoals
{
   public static YoVariableTestGoal notFallen(AvatarTestYoVariables variables)
   {
      return YoVariableTestGoal.deltaGreaterThan(variables.getPelvisZ(), variables.getMidFeetZUpZ(), variables.getDesiredCOMHeight().getDoubleValue() / 2.0);
   }
}
