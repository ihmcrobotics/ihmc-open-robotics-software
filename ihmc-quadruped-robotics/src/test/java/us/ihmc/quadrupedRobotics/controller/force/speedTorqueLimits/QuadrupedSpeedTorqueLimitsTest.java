package us.ihmc.quadrupedRobotics.controller.force.speedTorqueLimits;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedSpeedTorqueLimitsTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseStateEstimator(false);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 30000)
   public void testStandingLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);
      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 300000)
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

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
   public void testXGaitWalkingInPlaceLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
   public void testXGaitTrottingInPlaceLowerLimit(double nominalCoMHeight)
   {
      standupPrecisely(nominalCoMHeight);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      lowerHeightUntilFailure(nominalCoMHeight);

      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
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

   private void standupPrecisely(double desiredCoMHeight) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredBodyHeight(desiredCoMHeight);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getCurrentHeightInWorld(), desiredCoMHeight, 0.01));
      conductor.simulate();
   }

   private void lowerHeightUntilFailure(double originalHeight) throws AssertionFailedError
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
   
   private void raiseHeightUntilFailure(double originalHeight) throws AssertionFailedError
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
