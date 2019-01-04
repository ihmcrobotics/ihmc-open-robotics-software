package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedPlanning.input.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
//   private QuadrupedTeleopManager stepTeleopManager;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract double getPacingWidth();

   public abstract double getFastWalkingSpeed();
   public abstract double getSlowWalkingSpeed();
   public abstract double getWalkingAngularVelocity();
   public abstract double getWalkingSpeedWhileTurning();

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
//         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testWalkingForwardFast()
   {
      testFlatGroundWalking(90.0, getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testWalkingForwardSlow()
   {
      testFlatGroundWalking(90.0, getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testWalkingBackwardsFast()
   {
      testFlatGroundWalking(90.0, -getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testWalkingBackwardsSlow()
   {
      testFlatGroundWalking(90.0, -getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testWalkingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testWalkingInABackwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testTrottingForwardFast()
   {
      testFlatGroundWalking(180.0, getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testTrottingForwardSlow()
   {
      testFlatGroundWalking(180.0, getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testTrottingBackwardsFast()
   {
      testFlatGroundWalking(180.0, -getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testTrottingBackwardsSlow()
   {
      testFlatGroundWalking(180.0, -getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testTrottingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testTrottingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testTrottingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testTrottingInABackwardRightCircle()
   {
      testWalkingInASemiCircle(180.0, -getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   private void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setEndPhaseShift(endPhaseShift);

      double walkTime = 6.0;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));

      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getSteppingState(), QuadrupedSteppingStateEnum.STAND));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 0.5));

      conductor.simulate();

      QuadrupedTestBehaviors.sitDown(conductor, variables);
   }


   private void testWalkingInASemiCircle(double endPhaseShift, double walkingSpeed, double angularVelocity)
   {
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(walkingSpeed / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(walkingSpeed) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }

      conductor.simulate();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testPacingForwardFast()
   {
      testFlatGroundPacing(getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testPacingForwardSlow()
   {
      testFlatGroundPacing(getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testPacingBackwardsFast()
   {
      testFlatGroundPacing(-getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testPacingBackwardsSlow()
   {
      testFlatGroundPacing(-getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testPacingInAForwardLeftCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 246.9)
   @Test(timeout = 1200000)
   public void testPacingInAForwardRightCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testPacingInABackwardLeftCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testPacingInABackwardRightCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   private void testFlatGroundPacing(double walkingSpeed)
   {
      stepTeleopManager.setStanceWidth(getPacingWidth());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setEndPhaseShift(0.0);

      double walkTime = 5.0;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();
   }

   private void testPacingInASemiCircle(double walkingSpeed, double angularVelocity)
   {
      stepTeleopManager.setStanceWidth(getPacingWidth());

      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(walkingSpeed / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setEndPhaseShift(0.0);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(walkingSpeed) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }

      conductor.simulate();
   }
}
