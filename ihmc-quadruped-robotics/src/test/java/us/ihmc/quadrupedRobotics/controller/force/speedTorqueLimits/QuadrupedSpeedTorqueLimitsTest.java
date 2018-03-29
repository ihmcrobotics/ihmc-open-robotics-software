package us.ihmc.quadrupedRobotics.controller.force.speedTorqueLimits;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedSpeedTorqueLimitsTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedStepTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseStateEstimator(false);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
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
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 30000)
   public void testStandingLowerLimit()
   {
      double originalHeight = standupPrecisely();

      lowerHeightUntilFailure(originalHeight);

      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 300000)
   public void testStandingOnThreeLegsLowerLimit()
   {
      double originalHeight = standupPrecisely();
      
      variables.getTimedStepQuadrant().set(RobotQuadrant.FRONT_LEFT);
      variables.getTimedStepGroundClearance().set(0.2);
      variables.getTimedStepDuration().set(30.0);
      variables.getTimedStepGoalPositionX().set(variables.getSolePositionXs().get(RobotQuadrant.FRONT_LEFT).getDoubleValue());
      variables.getTimedStepGoalPositionY().set(variables.getSolePositionYs().get(RobotQuadrant.FRONT_LEFT).getDoubleValue() + 0.2);
      variables.getTimedStepGoalPositionZ().set(0.2);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);

      lowerHeightUntilFailure(originalHeight);
      raiseHeightUntilFailure(originalHeight);

      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
   public void testXGaitWalkingInPlaceLowerLimit()
   {
      double originalHeight = standupPrecisely();
      
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      lowerHeightUntilFailure(originalHeight);

      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
   public void testXGaitTrottingInPlaceLowerLimit()
   {
      double originalHeight = standupPrecisely();
      
      QuadrupedTestBehaviors.enterXGait(conductor, variables, stepTeleopManager);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      lowerHeightUntilFailure(originalHeight);

      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 30000)
   public void testXGaitWalkingLowerLimit()
   {
      double originalHeight = standupPrecisely();
      
      QuadrupedTestBehaviors.enterXGait(conductor, variables, stepTeleopManager);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.7, 0.0, 0.0);

      lowerHeightUntilFailure(originalHeight);

      conductor.concludeTesting();
   }

   private double standupPrecisely() throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      double originalHeight = variables.getYoComPositionInputZ().getDoubleValue();
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), originalHeight, 0.01));
      conductor.simulate();
      return originalHeight;
   }

   private void lowerHeightUntilFailure(double originalHeight) throws AssertionFailedError
   {
      for (double heightDelta = 0.0; (originalHeight + heightDelta) > 0.38; heightDelta -= 0.01)
      {
         variables.getYoComPositionInputZ().set(originalHeight + heightDelta);

         variables.getLimitJointTorques().set(false);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), originalHeight + heightDelta, 0.01));
         conductor.simulate();

         try
         {
            variables.getLimitJointTorques().set(true);
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addSustainGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), originalHeight + heightDelta, 0.01));
            conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
            conductor.simulate();
         }
         catch (AssertionFailedError assertionFailedError)
         {
            PrintTools.info("Failed to stand at " + variables.getYoComPositionInputZ().getDoubleValue());
            break;
         }
      }
   }
   
   private void raiseHeightUntilFailure(double originalHeight) throws AssertionFailedError
   {
      for (double heightDelta = 0.38 - originalHeight; (originalHeight + heightDelta) < originalHeight; heightDelta += 0.01)
      {
         variables.getYoComPositionInputZ().set(originalHeight + heightDelta);

         variables.getLimitJointTorques().set(false);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), originalHeight + heightDelta, 0.01));
         conductor.simulate();

         try
         {
            variables.getLimitJointTorques().set(true);
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addSustainGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), originalHeight + heightDelta, 0.01));
            conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
            conductor.simulate();
         }
         catch (AssertionFailedError assertionFailedError)
         {
            PrintTools.info("Failed to stand at " + variables.getYoComPositionInputZ().getDoubleValue());
            break;
         }
      }
   }
}
