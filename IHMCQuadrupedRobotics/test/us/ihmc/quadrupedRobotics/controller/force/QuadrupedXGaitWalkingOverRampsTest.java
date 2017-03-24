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
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.simulationconstructionsettools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      ParameterRegistry.destroyAndRecreateInstance();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.075, 0.75, 1.2);
      
      walkOverRamps(groundProfile);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 100000)
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
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());

      variables.getXGaitEndDoubleSupportDurationInput().set(0.05);
      variables.getXGaitStanceLengthInput().set(1.00);
      variables.getXGaitStanceWidthInput().set(0.30);
      variables.getXGaitStepDurationInput().set(0.35);
      variables.getXGaitStepGroundClearanceInput().set(0.1);
      variables.getYoComPositionInputZ().set(0.575);

      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.5);
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(-0.5);
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingDownSlope() throws IOException
   {
      walkSlope(0.2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingUpSlope() throws IOException
   {
      walkSlope(-0.1);
   }

   private void walkSlope(double slope) throws IOException, AssertionFailedError
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(slope);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseStateEstimator(false);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());

      variables.getXGaitEndDoubleSupportDurationInput().set(0.05);
      variables.getXGaitStanceLengthInput().set(1.00);
      variables.getXGaitStanceWidthInput().set(0.30);
      variables.getXGaitStepDurationInput().set(0.35);
      variables.getXGaitStepGroundClearanceInput().set(0.1);
      variables.getYoComPositionInputZ().set(0.575);

      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.75);
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(-0.75);
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
