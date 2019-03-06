package us.ihmc.atlas.behaviors;

import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

public class AtlasTestScripts
{
   public static void standUp(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables)
   {
      // 1.8 seconds hand measured for Atlas pelvis to settle
      conductor.addDurationGoal(variables.getYoTime(), 1.8);  // TODO Improve by detecting stablilization
      conductor.simulate();
   }

   public static void nextTouchdown(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables, double timeLimit)
   {
      conductor.addTerminalGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), false));
      conductor.addTimeLimit(variables.getYoTime(), timeLimit);
      conductor.simulate();
      conductor.addTerminalGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), true));
      conductor.addTimeLimit(variables.getYoTime(), timeLimit);
      conductor.simulate();
   }
}
