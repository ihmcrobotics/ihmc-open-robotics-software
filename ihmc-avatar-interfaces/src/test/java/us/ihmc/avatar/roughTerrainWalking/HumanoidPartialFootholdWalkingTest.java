package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class HumanoidPartialFootholdWalkingTest implements MultiRobotTestInterface
{
   private final static boolean visualize = true;
   private final static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   @Test
   public void testPartialFootholdField() throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.25;
      double stepLength = 0.3;

      double topHeight = 0.1;
      PartialFootholdField environment = new PartialFootholdField(0.22, stepWidth, stepLength, topHeight, 0.6);

      setupTest(environment);



      drcSimulationTestHelper.publishToController(environment.getSteps());
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(environment.getSteps().getFootstepDataList().size() * 1.5 + 2.0);
      assertTrue(success);
   }


   @Test
   public void testSteppingOntoWithInsideOfFoot() throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.25;

      double blockWidth = 0.4;
      double blockDepth = 0.5;
      double blockDistanceFromOrigin = 0.25;
      double blockY = 0.5 * (stepWidth - blockWidth);
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + 0.5 * blockDepth, blockY,  blockDepth, blockWidth, topHeight);

      setupTest(environment);


      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.15, stepWidth / 2, topHeight);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }

   @Test
   public void testSteppingOntoWithOutsideOfFoot() throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.25;

      double blockWidth = 0.4;
      double blockDepth = 0.5;
      double blockDistanceFromOrigin = 0.25;
      double blockY = 0.5 * (stepWidth + blockWidth);
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + 0.5 * blockDepth, blockY,  blockDepth, blockWidth, topHeight);

      setupTest(environment);


      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.15, stepWidth / 2, topHeight);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }

   @Test
   public void testSteppingOntoBlock() throws SimulationExceededMaximumTimeException
   {
      double blockWidth = 0.3;
      double blockDistanceFromOrigin = 0.2;
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + blockWidth * 0.5, blockWidth, 0.4, topHeight);

      setupTest(environment);

      double width = 0.25;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.05, width / 2, topHeight);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }

   @Test
   public void testSteppingOntoNarrowBlock() throws SimulationExceededMaximumTimeException
   {
      double blockDepth = 0.05;
      double blockWidth = 0.4;
      double stepDistance = 0.25;
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(stepDistance, blockDepth, blockWidth, topHeight);

      setupTest(environment);

      double width = 0.25;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(stepDistance, width / 2, topHeight);

      FootstepDataMessage step2 = message.getFootstepDataList().add();
      step2.setRobotSide(FootstepDataMessage.ROBOT_SIDE_RIGHT);
      step2.getLocation().set(2 * stepDistance, -width / 2, 0.0);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }

   @Test
   public void testWalkingOverBlock() throws SimulationExceededMaximumTimeException
   {
      double blockWidth = 0.3;
      double blockDistanceFromOrigin = 0.2;
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + blockWidth * 0.5, blockWidth, 0.4, topHeight);

      setupTest(environment);

      double width = 0.25;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.045, width / 2, topHeight);

      FootstepDataMessage step2 = message.getFootstepDataList().add();
      step2.setRobotSide(FootstepDataMessage.ROBOT_SIDE_RIGHT);
      step2.getLocation().set(blockDistanceFromOrigin + 0.3, -width / 2, topHeight);

      FootstepDataMessage step3 = message.getFootstepDataList().add();
      step3.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step3.getLocation().set(blockDistanceFromOrigin + 0.55, width / 2, 0.0);

      FootstepDataMessage step4 = message.getFootstepDataList().add();
      step4.setRobotSide(FootstepDataMessage.ROBOT_SIDE_RIGHT);
      step4.getLocation().set(blockDistanceFromOrigin + 0.55, -width / 2, 0.0);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }




   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest(CommonAvatarEnvironmentInterface environment) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      String className = getClass().getSimpleName();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(className);

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      YoDouble damping_l_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      YoDouble damping_r_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      YoDouble damping_r_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
   }


   public class SimpleBlockEnvironment extends PlanarRegionEnvironmentInterface
   {
      public SimpleBlockEnvironment(double blockPosition, double blockDepth, double blockWidth, double blockHeight)
      {
         this(blockPosition, 0.0, blockDepth, blockWidth, blockHeight);
      }

      public SimpleBlockEnvironment(double blockXPosition, double blockYPosition, double blockDepth, double blockWidth, double blockHeight)
      {
         this(blockXPosition, blockYPosition, 0.0, blockDepth, blockWidth, blockHeight);
      }

      public SimpleBlockEnvironment(double blockXPosition, double blockYPosition, double blockYaw, double blockDepth, double blockWidth, double blockHeight)
      {
         // first ground plane
         generator.identity();
         generator.addRectangle(10.0, 10.0);

         generator.translate(blockXPosition, blockYPosition, 0.0);
         generator.rotateEuler(new Vector3D(0.0, 0.0, blockYaw));

         generator.addCubeReferencedAtBottomMiddle(blockDepth, blockWidth, blockHeight);

         addPlanarRegionsToTerrain(YoAppearance.Grey());
      }
   }

   public class PartialFootholdField extends PlanarRegionEnvironmentInterface
   {
      private final FootstepDataListMessage steps = new FootstepDataListMessage();

      public PartialFootholdField(double footLength, double stanceWidth, double stepLength, double blockHeight, double fractionOfFoothold)
      {
         generator.identity();
         generator.addRectangle(10.0, 10.0);

         // first step with toes on
         FootstepDataMessage step = steps.getFootstepDataList().add();
         step.getLocation().set(stepLength, 0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.LEFT.toByte());

         double blockDepth = stepLength + 2.0 * (fractionOfFoothold - 0.5) * footLength;
         generator.translate(1.5 * stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(blockDepth, 0.5, blockHeight);

         // second step with heel on
         step = steps.getFootstepDataList().add();
         step.getLocation().set(2.0 * stepLength, -0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.RIGHT.toByte());

         // third step with outside of foot on
         double blockWidth = 0.4;
         generator.translate(1.5 * stepLength, 0.5 * (stanceWidth + blockWidth), 0.0);
         generator.addCubeReferencedAtBottomMiddle(1.5 * footLength, blockWidth, blockHeight);

         step = steps.getFootstepDataList().add();
         step.getLocation().set(3.0 * stepLength, 0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.LEFT.toByte());

         // fourth set with inside of foot on
         generator.translate(stepLength, -stanceWidth, 0.0);
         generator.addCubeReferencedAtBottomMiddle(1.5 * footLength, blockWidth, blockHeight);

         step = steps.getFootstepDataList().add();
         step.getLocation().set(4.0 * stepLength, -0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.RIGHT.toByte());

         generator.translate(0.0, -0.25 * stanceWidth, 0.0);
         generator.translate(1.5 * stepLength, 0.0, 0.0);
         generator.rotateEuler(new Vector3D(0.0, 0.0, Math.PI / 4.0));
         generator.addCubeReferencedAtBottomMiddle(stepLength / Math.sin(Math.PI / 4.0), stepLength / Math.sin(Math.PI / 4.0), blockHeight);

         step = steps.getFootstepDataList().add();
         step.getLocation().set(5.0 * stepLength, 0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.LEFT.toByte());

         step = steps.getFootstepDataList().add();
         step.getLocation().set(6.0 * stepLength, -0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.RIGHT.toByte());

         generator.identity();
         generator.translate(7.0 * stepLength, 0.5 * stanceWidth, 0.0);
         generator.addCubeReferencedAtBottomMiddle(0.5 * footLength, 0.3, blockHeight);

         step = steps.getFootstepDataList().add();
         step.getLocation().set(7.0 * stepLength, 0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.LEFT.toByte());

         generator.translate(1.05 * stepLength, -1.5 * stanceWidth, 0.0);
         generator.rotateEuler(new Vector3D(0.0, 0.0, Math.PI / 4.0));
         double sideWidth = 0.3;
         generator.addCubeReferencedAtBottomMiddle(sideWidth, sideWidth, blockHeight);

         step = steps.getFootstepDataList().add();
         step.getLocation().set(8.0 * stepLength, -0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.RIGHT.toByte());

         step = steps.getFootstepDataList().add();
         step.getLocation().set(9.0 * stepLength, 0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.LEFT.toByte());

         step = steps.getFootstepDataList().add();
         step.getLocation().set(8.0 * stepLength, -0.5 * stanceWidth, blockHeight);
         step.setRobotSide(RobotSide.RIGHT.toByte());

         addPlanarRegionsToTerrain(YoAppearance.Grey());
      }

      public FootstepDataListMessage getSteps()
      {
         return steps;
      }
   }
}
