package us.ihmc.avatar.jumping;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.robotics.Assert.*;

public abstract class AvatarStandingLongJumpTests
{
   private static final double goalEpsilon = 1e-1;
   private static final double shortJumpLength = 0.25;
   private static final double mediumJumpLength = 0.5;
   private static final double longJumpLength = 0.75;

   private static final boolean visualize = false;
   private static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private JumpingTestHelper testHelper;

   protected abstract DRCRobotModel getRobotModel();

   protected abstract DRCRobotInitialSetup getInitialSetup();

   protected abstract String getParameterResourceName();

   @BeforeEach
   public void create()
   {
      simulationTestingParameters.setKeepSCSUp(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      testHelper = new JumpingTestHelper(simulationTestingParameters, getRobotModel(), getInitialSetup(), getParameterResourceName());
   }

   @AfterEach
   public void destroy()
   {
      testHelper.destroySimulation();
      testHelper = null;
   }

   @Test
   public void testStandingShortJump() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      runTest(shortJumpLength, 0.4);
   }

   @Test
   public void testStandingMediumJump() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      runTest(mediumJumpLength, 0.3);
   }

   @Disabled
   @Test
   public void testStandingLongJump() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      runTest(longJumpLength, 0.3);
   }

   private void runTest(double jumpLength, double supportDuration) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      YoRegistry registry = testHelper.getSCS().getRootRegistry();
      YoDouble comX = ((YoDouble) registry.findVariable("centerOfMassX"));
      YoDouble comY = ((YoDouble) registry.findVariable("centerOfMassY"));
      YoDouble comZ = ((YoDouble) registry.findVariable("centerOfMassZ"));

      testHelper.startSimulation();
      BlockingSimulationRunner blockingSimulationRunner = testHelper.getBlockingSimulationRunner();

      boolean success = blockingSimulationRunner.simulateAndBlockAndCatchExceptions(1.0);
      testHelper.triggerSquat(true);
      success &= blockingSimulationRunner.simulateAndBlockAndCatchExceptions(0.5);

      JumpingGoal jumpingGoal = new JumpingGoal();
      jumpingGoal.setGoalLength(jumpLength);
      jumpingGoal.setSupportDuration(supportDuration);
      jumpingGoal.setFlightDuration(0.25);
      testHelper.submitCommand(jumpingGoal);

      success &= blockingSimulationRunner.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      assertEquals(jumpLength, comX.getDoubleValue(), goalEpsilon);
      assertEquals(0.0, comY.getDoubleValue(), goalEpsilon);
   }

}
