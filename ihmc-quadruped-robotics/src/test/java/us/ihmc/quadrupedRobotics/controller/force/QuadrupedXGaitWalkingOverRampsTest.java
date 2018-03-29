package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private QuadrupedStepTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testWalkingOverShallowRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.075, 0.75, 1.2);
      
      walkOverRamps(groundProfile);
   }

   public void testWalkingOverAggressiveRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.15, 0.75, 1.2);
      
      walkOverRamps(groundProfile);
   }

   private void walkOverRamps(RampsGroundProfile groundProfile) throws IOException, AssertionFailedError
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.05);
      stepTeleopManager.getXGaitSettings().setStanceLength(1.00);
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.30);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.35);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.1);
      variables.getYoComPositionInputZ().set(0.575);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      QuadrupedTestBehaviors.enterXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(-0.5, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   public void testWalkingDownSlope(QuadrupedSimulationInitialPositionParameters initialPosition) throws IOException
   {
      walkSlope(0.2, initialPosition);
   }

   public void testWalkingUpSlope(QuadrupedSimulationInitialPositionParameters initialPosition) throws IOException
   {
      walkSlope(-0.1, initialPosition);
   }

   private void walkSlope(double slope, QuadrupedSimulationInitialPositionParameters initialPosition) throws IOException, AssertionFailedError
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(slope);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setInitialPosition(initialPosition);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseStateEstimator(false);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      QuadrupedTestBehaviors.enterXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.75, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 6.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 3.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(-0.75, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
