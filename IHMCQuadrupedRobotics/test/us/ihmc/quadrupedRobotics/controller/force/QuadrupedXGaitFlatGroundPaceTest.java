package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionsettools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitFlatGroundPaceTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         ParameterRegistry.destroyAndRecreateInstance();
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseStateEstimator(false);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
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
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testPacingForwardFast()
   {
      paceFast(1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testPacingBackwardsFast()
   {
      paceFast(-1.0);
   }

   private void paceFast(double directionX) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(0.0);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.getYoPlanarVelocityInputX().set(directionX * 1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 10.0);
      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX * 2.0));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 2.0));
      }
      conductor.simulate();
      
      conductor.concludeTesting(2);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testPacingForwardSlow()
   {
      paceSlow(1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testPacingBackwardsSlow()
   {
      paceSlow(-1.0);
   }

   private void paceSlow(double directionX) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(0.0);
      variables.getXGaitEndDoubleSupportDurationInput().set(0.3);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.getYoPlanarVelocityInputX().set(directionX * 0.1);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 10.0);
      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX * 0.4));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 0.4));
      }
      conductor.simulate();
      
      conductor.concludeTesting(2);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testPacingInAForwardLeftCircle()
   {
      paceInACircle(1.0, 1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testPacingInAForwardRightCircle()
   {
      paceInACircle(1.0, -1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testPacingInABackwardLeftCircle()
   {
      paceInACircle(-1.0, 1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testPacingInABackwardRightCircle()
   {
      paceInACircle(-1.0, -1.0);
   }

   private void paceInACircle(double directionX, double directionZ) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(0.0);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(directionX * 0.6);
      variables.getYoPlanarVelocityInputZ().set(directionZ * 0.5);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 10.0);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 0.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), directionZ * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), directionZ * Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting(2);
   }
}
