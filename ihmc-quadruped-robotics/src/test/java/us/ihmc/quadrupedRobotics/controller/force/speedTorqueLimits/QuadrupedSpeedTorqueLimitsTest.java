package us.ihmc.quadrupedRobotics.controller.force.speedTorqueLimits;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.commons.PrintTools;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedSpeedTorqueLimitsTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseStateEstimator(false);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testStandingLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);
      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @Test
   public void testStandingOnThreeLegsLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);

      variables.getTimedStepQuadrant().set(RobotQuadrant.FRONT_LEFT);
      variables.getTimedStepGroundClearance().set(0.2);
      variables.getTimedStepDuration().set(30.0);
      variables.getTimedStepGoalPositionX().set(variables.getSolePositionXs().get(RobotQuadrant.FRONT_LEFT).getDoubleValue());
      variables.getTimedStepGoalPositionY().set(variables.getSolePositionYs().get(RobotQuadrant.FRONT_LEFT).getDoubleValue() + 0.2);
      variables.getTimedStepGoalPositionZ().set(0.2);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);

      lowerHeightUntilFailure(nominalCoMHeight);
      raiseHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @Test
   public void testXGaitWalkingInPlaceLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @Test
   public void testXGaitTrottingInPlaceLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      stepTeleopManager.setEndPhaseShift(180.0);
      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @Test
   public void testXGaitWalkingLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.7, 0.0, 0.0);

      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   private void standupPrecisely(double desiredCoMHeight)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredBodyHeight(desiredCoMHeight);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), desiredCoMHeight, 0.01));
      conductor.simulate();
   }

   private void lowerHeightUntilFailure(double originalHeight)
   {
      for (double heightDelta = 0.0; (originalHeight + heightDelta) > 0.38; heightDelta -= 0.01)
      {
         double desiredCoMHeight = originalHeight + heightDelta;
         stepTeleopManager.setDesiredBodyHeight(desiredCoMHeight);

         variables.getLimitJointTorques().set(false);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), originalHeight + heightDelta, 0.01));
         conductor.simulate();

         try
         {
            variables.getLimitJointTorques().set(true);
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addSustainGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), originalHeight + heightDelta, 0.01));
            conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
            conductor.simulate();
         }
         catch (AssertionFailedError assertionFailedError)
         {
            PrintTools.info("Failed to stand at " + desiredCoMHeight);
            break;
         }
      }
   }

   private void raiseHeightUntilFailure(double originalHeight)
   {
      for (double heightDelta = 0.38 - originalHeight; (originalHeight + heightDelta) < originalHeight; heightDelta += 0.01)
      {
         double desiredCoMHeight = originalHeight + heightDelta;
         stepTeleopManager.setDesiredBodyHeight(desiredCoMHeight);

         variables.getLimitJointTorques().set(false);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), originalHeight + heightDelta, 0.01));
         conductor.simulate();

         try
         {
            variables.getLimitJointTorques().set(true);
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addSustainGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), originalHeight + heightDelta, 0.01));
            conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
            conductor.simulate();
         }
         catch (AssertionFailedError assertionFailedError)
         {
            PrintTools.info("Failed to stand at " + desiredCoMHeight);
            break;
         }
      }
   }
}
