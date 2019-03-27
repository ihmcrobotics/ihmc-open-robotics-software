package us.ihmc.atlas.behaviors;

import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

public class AtlasTestScripts
{
   public static void standUp(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables)
   {
      LogTools.info("Waiting for robot to stand");
      // 1.8 seconds hand measured for Atlas pelvis to settle
      conductor.addDurationGoal(variables.getYoTime(), 1.8);  // TODO Improve by detecting stablilization
      conductor.simulate();
   }

   public static void nextTouchdown(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables, double timeLimit)
   {
      conductor.addTerminalGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), false));
      conductor.addTimeLimit(variables.getYoTime(), timeLimit / 2);
      conductor.simulate();
      conductor.addTerminalGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), true));
      conductor.addTimeLimit(variables.getYoTime(), timeLimit / 2);
      conductor.simulate();
   }

   public static void holdDoubleSupport(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables, double holdDuration, double timeLimit)
   {
      LogTools.info("Waiting for double support", holdDuration);
      conductor.addTerminalGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), true));
      conductor.addTimeLimit(variables.getYoTime(), timeLimit);
      conductor.simulate();
      LogTools.info("Asserting double support to holds for {}", holdDuration);
      conductor.addSustainGoal(YoVariableTestGoal.booleanEquals(variables.getControllerIsInDoubleSupport(), true));
      conductor.addDurationGoal(variables.getYoTime(), holdDuration);
      conductor.simulate();
   }

   public static void wait(GoalOrientedTestConductor conductor, AvatarTestYoVariables variables, double duration)
   {
      LogTools.info("Waiting for {} s", duration);
      conductor.addDurationGoal(variables.getYoTime(), duration);
      conductor.simulate();
   }
}
