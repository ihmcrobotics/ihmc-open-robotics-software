package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

@Tag("quadruped-xgait-2")
public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract double getPacingWidth();
   public abstract double getPacingStepDuration();
   public abstract double getPacingEndDoubleSupportDuration();

   public abstract double getFastWalkingSpeed();
   public abstract double getSlowWalkingSpeed();
   public abstract double getWalkingAngularVelocity();
   public abstract double getWalkingSpeedWhileTurning();

   @BeforeEach
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testWalkingForwardFast()
   {
      testFlatGroundWalking(90.0, getFastWalkingSpeed());
   }

   @Test
   public void testWalkingForwardSlow()
   {
      testFlatGroundWalking(90.0, getSlowWalkingSpeed());
   }

   @Test
   public void testWalkingBackwardsFast()
   {
      testFlatGroundWalking(90.0, -getFastWalkingSpeed());
   }

   @Test
   public void testWalkingBackwardsSlow()
   {
      testFlatGroundWalking(90.0, -getSlowWalkingSpeed());
   }

   @Test
   public void testWalkingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @Test
   public void testWalkingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
   public void testWalkingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
   public void testWalkingInABackwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   @Test
   public void testTrottingForwardFast()
   {
      testFlatGroundWalking(180.0, getFastWalkingSpeed());
   }

   @Test
   public void testTrottingForwardSlow()
   {
      testFlatGroundWalking(180.0, getSlowWalkingSpeed());
   }

   @Test
   public void testTrottingBackwardsFast()
   {
      testFlatGroundWalking(180.0, -getFastWalkingSpeed());
   }

   @Test
   public void testTrottingBackwardsSlow()
   {
      testFlatGroundWalking(180.0, -getSlowWalkingSpeed());
   }

   @Test
   public void testTrottingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @Test
   public void testTrottingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
   public void testTrottingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
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

   @Test
   public void testPacingForwardFast()
   {
      testFlatGroundPacing(getFastWalkingSpeed());
   }

   @Test
   public void testPacingForwardSlow()
   {
      testFlatGroundPacing(getSlowWalkingSpeed());
   }

   @Test
   public void testPacingBackwardsFast()
   {
      testFlatGroundPacing(-getFastWalkingSpeed());
   }

   @Test
   public void testPacingBackwardsSlow()
   {
      testFlatGroundPacing(-getSlowWalkingSpeed());
   }

   @Test
   public void testPacingInAForwardLeftCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @Test
   public void testPacingInAForwardRightCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
   public void testPacingInABackwardLeftCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @Test
   public void testPacingInABackwardRightCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   private void testFlatGroundPacing(double walkingSpeed)
   {
      stepTeleopManager.setStanceWidth(getPacingWidth());
      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      stepTeleopManager.setStepDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.PACE.getEndPhaseShift(), getPacingStepDuration());
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.PACE.getEndPhaseShift(), getPacingEndDoubleSupportDuration());

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
      stepTeleopManager.setStepDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.PACE.getEndPhaseShift(), getPacingStepDuration());
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.PACE.getEndPhaseShift(), getPacingEndDoubleSupportDuration());

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
