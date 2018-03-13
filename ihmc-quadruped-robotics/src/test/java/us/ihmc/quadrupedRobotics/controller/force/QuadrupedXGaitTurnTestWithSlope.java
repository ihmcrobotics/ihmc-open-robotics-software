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

public abstract class QuadrupedXGaitTurnTestWithSlope implements QuadrupedMultiRobotTestInterface
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
         InclinedGroundProfile groundProfile = new InclinedGroundProfile(0.1);

         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setGroundProfile3D(groundProfile);
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
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
   public void testTrottingDiagonallyForwardLeftWithSlope()throws IOException { trottingDiagonally(1,0.3, 180.0);}

   @Test
   public void testTrottingDiagonallyBackwardLeftWithSlope()throws IOException { trottingDiagonally(-1,0.3, 180.0);}

   @Test
   public void testTrottingDiagonallyForwardRightWithSlope() throws IOException{ trottingDiagonally(1,-0.3, 180.0);}

   @Test
   public void testTrottingDiagonallyBackwardRightWithSlope()throws IOException { trottingDiagonally(-1,-0.3, 180.0);}


   private void trottingDiagonally(double directionX, double directionY, double endPhaseShiftInput) throws IOException,AssertionFailedError
   {


      variables.getXGaitEndDoubleSupportDurationInput().set(0.05);
      variables.getXGaitStanceLengthInput().set(1.00);
      variables.getXGaitStanceWidthInput().set(0.30);
      variables.getXGaitStepDurationInput().set(0.35);
      variables.getXGaitStepGroundClearanceInput().set(0.1);
      variables.getYoComPositionInputZ().set(0.575);
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
   public void testTrottingForwardThenTurnLeftWithSlope() throws IOException { gaitThenTurn(1, 1, 180.0);}

   @Test
   public void testTrottingBackwardThenTurnLeftWithSlope() throws IOException { gaitThenTurn(-1, 1, 180.0);}

   @Test
   public void testTrottingForwardThenTurnRightWithSlope() throws IOException { gaitThenTurn(1, -1, 180.0);}

   @Test
   public void testTrottingBackwardThenTurnRightWithSlope()throws IOException { gaitThenTurn(-1, -1, 180.0);}


   private void gaitThenTurn(double directionX, double directionY, double endPhaseShiftInput) throws IOException, AssertionFailedError
   {

      variables.getXGaitEndDoubleSupportDurationInput().set(0.05);
      variables.getXGaitStanceLengthInput().set(1.00);
      variables.getXGaitStanceWidthInput().set(0.30);
      variables.getXGaitStepDurationInput().set(0.35);
      variables.getXGaitStepGroundClearanceInput().set(0.1);
      variables.getYoComPositionInputZ().set(0.575);

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
