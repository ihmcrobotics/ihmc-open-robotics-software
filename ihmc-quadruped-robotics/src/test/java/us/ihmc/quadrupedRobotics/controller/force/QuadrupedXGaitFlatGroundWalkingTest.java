package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;

import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract double getPacingWidth();

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
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);

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

   public void testFlatGroundPacing(double walkingSpeed)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(getPacingWidth());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0.0);

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

   public void testWalkingInASemiCircle(double endPhaseShift, double forwardVelocity, double angularVelocity)
   {
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(forwardVelocity / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setDesiredVelocity(forwardVelocity, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(forwardVelocity) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * forwardVelocity * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * forwardVelocity * 0.6));
      }

      conductor.simulate();
   }

   public void testPacingInASemiCircle(double forwardVelocity, double angularVelocity)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(getPacingWidth());

      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(forwardVelocity / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0.0);
      stepTeleopManager.setDesiredVelocity(forwardVelocity, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(forwardVelocity) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * forwardVelocity * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * forwardVelocity * 0.6));
      }

      conductor.simulate();
   }
}
