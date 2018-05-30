package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class Quadruped2018PIDemoTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
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
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double initialDoubleSupportDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();
      double singleSupportDuration = 0.3;

      stepTeleopManager.setDesiredCoMHeight(0.8);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.15);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.05);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5 * walkingSpeed, 0.0, 0.0);

      double initialWalkTime = initialDoubleSupportDuration + singleSupportDuration;
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), initialWalkTime));

      double walkTime = 5.0;
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.01);
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

}
