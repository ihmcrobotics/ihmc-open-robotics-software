package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedPlanning.footstepChooser.HeightMapFootSnapper;
import us.ihmc.quadrupedPlanning.input.OldQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitBumpyTerrainWalkingTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private OldQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 520000)
   public void testWalkingOverShallowBumpyTerrain() throws IOException
   {
      testWalkingOverShallowBumpyTerrain(1.0);
   }

   protected void testWalkingOverShallowBumpyTerrain(double speed) throws IOException
   {
      double xAmp1 = 0.01, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.01, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, 1.2);

      setup(groundProfile);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(speed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 2.0));
      conductor.simulate();
   }


   private void setup(BumpyGroundProfile groundProfile) throws IOException
   {
      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 650000)
   public void testWalkingOverMediumBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      double xAmp1 = 0.02, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.02, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, 1.2);
      setup(groundProfile);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.1);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 15.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 2.0));
      conductor.simulate();
   }

   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 350000)
   public void testTrottingOverAggressiveBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      double xAmp1 = 0.03, xFreq1 = 0.5, xAmp2 = 0.02, xFreq2 = 0.5;
      double yAmp1 = 0.02, yFreq1 = 0.07, yAmp2 = 0.02, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, 1.2);
      setup(groundProfile);
      stepTeleopManager.setStepSnapper(new HeightMapFootSnapper(groundProfile));

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.5));
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.1);
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.35);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.35);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.1);
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();
   }
}
