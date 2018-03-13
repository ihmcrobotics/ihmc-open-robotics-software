package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitChangeDirectionTest implements QuadrupedMultiRobotTestInterface
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
      conductor.concludeTesting(2);
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testTrottingForwardThenChangeToLeft() { gaitThenChangeDirection(1, 1, 180.0);}

   @Test
   public void testTrottingBackwardThenChangeToLeft() { gaitThenChangeDirection(-1, 1, 180.0);}

   @Test
   public void testTrottingForwardThenChangeToRight() { gaitThenChangeDirection(1, -1, 180.0);}

   @Test
   public void testTrottingBackwardThenChangeToRight() { gaitThenChangeDirection(-1, -1, 180.0);}


   @Test
   public void testPacingForwardThenChangeToLeft() { gaitThenChangeDirection(1, 1, 0.0);}

   @Test
   public void testPacingBackwardThenChangeToLeft() { gaitThenChangeDirection(-1, 1, 0.0);}

   @Test
   public void testPacingForwardThenChangeToRight() { gaitThenChangeDirection(1, -1, 0.0);}

   @Test
   public void testPacingBackwardThenChangeToRight() { gaitThenChangeDirection(-1, -1, 0.0);}


   private void gaitThenChangeDirection(double directionX, double directionY, double endPhaseShiftInput) throws  AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);


      variables.getYoPlanarVelocityInputX().set(directionX * 1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX ));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX ));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      variables.getYoPlanarVelocityInputY().set(directionY*0.4);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);

      if(directionY < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyY(), directionY ));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyY(), directionY ));
      }
      conductor.simulate();
   }

}
