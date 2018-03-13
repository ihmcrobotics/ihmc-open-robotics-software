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
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitTurnTest implements QuadrupedMultiRobotTestInterface
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
   public void testTrottingDiagonallyForwardLeft() { trottingDiagonally(1,0.3, 0.0);}

   @Test
   public void testTrottingDiagonallyBackwardLeft() { trottingDiagonally(-1,0.3, 0.0);}

   @Test
   public void testTrottingDiagonallyForwardRight() { trottingDiagonally(1,-0.3, 0.0);}

   @Test
   public void testTrottingDiagonallyBackwardRight() { trottingDiagonally(-1,-0.3, 0.0);}


   @Test
   public void testPacingDiagonallyForwardLeft() { trottingDiagonally(1,0.3, 180.0);}

   @Test
   public void testPacingDiagonallyBackwardLeft() { trottingDiagonally(-1,0.3, 180.0);}

   @Test
   public void testPacingDiagonallyForwardRight() { trottingDiagonally(1,-0.3, 180.0);}

   @Test
   public void testPacingDiagonallyBackwardRight() { trottingDiagonally(-1,-0.3, 180.0);}


   private void trottingDiagonally(double directionX, double directionY, double endPhaseShiftInput) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.getYoPlanarVelocityInputX().set(directionX);
      variables.getYoPlanarVelocityInputY().set(directionY);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX*2 ));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX*2 ));
      }
      conductor.simulate();
   }

   @Test
   public void testTrottingForwardThenTurnLeft() { gaitThenTurn(1, 1, 180.0);}

   @Test
   public void testTrottingBackwardThenTurnLeft() { gaitThenTurn(-1, 1, 180.0);}

   @Test
   public void testTrottingForwardThenTurnRight() { gaitThenTurn(1, -1, 180.0);}

   @Test
   public void testTrottingBackwardThenTurnRight() { gaitThenTurn(-1, -1, 180.0);}


   @Test
   public void testPacingForwardThenTurnLeft() { gaitThenTurn(1, 1, 0.0);}

   @Test
   public void testPacingBackwardThenTurnLeft() { gaitThenTurn(-1, 1, 0.0);}

   @Test
   public void testPacingForwardThenTurnRight() { gaitThenTurn(1, -1, 0.0);}

   @Test
   public void testPacingBackwardThenTurnRight() { gaitThenTurn(-1, -1, 0.0);}


   private void gaitThenTurn(double directionX, double directionY, double endPhaseShiftInput) throws  AssertionFailedError
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

      variables.getYoPlanarVelocityInputZ().set(directionX*directionY*0.4);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);

      if(directionX *directionY< 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyYaw(), directionX*directionY *0.5* Math.PI));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyYaw(), directionX*directionY*0.5* Math.PI));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputZ().set(0.0);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      variables.getYoPlanarVelocityInputX().set(directionX*1.0);
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
