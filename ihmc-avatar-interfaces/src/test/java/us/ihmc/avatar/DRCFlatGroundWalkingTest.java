package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;

import org.junit.jupiter.api.*;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2RunsSameWayTwiceVerifier;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.MeshTerrainEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.MeshTerrainObjectFactory;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

@Tag("video")
public abstract class DRCFlatGroundWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   /**
    * TODO Need to implement a specific test for that. As the footstep generator for flat ground
    * walking keeps changing the upcoming footsteps on the fly, the ICP planner ends up creating
    * discontinuities. But this is an expected behavior.
    */
   private static final boolean CHECK_ICP_CONTINUITY = false;

   private static final double yawingTimeDuration = 0.5;
   private static final double standingTimeDuration = RigidBodyControlManager.INITIAL_GO_HOME_TIME;
   private static final double defaultWalkingTimeDuration = BambooTools.isEveryCommitBuild() ? 45.0 : 90.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static String physicsEngineName;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Override
   public abstract DRCRobotModel getRobotModel();

   public abstract boolean doPelvisWarmup();

   public boolean getUsePerfectSensors()
   {
      return false;
   }

   @Tag("humanoid-flat-ground")
   @Test
   public void testFlatGroundWalking()
   {
      runFlatGroundWalking(false);
   }

   @Disabled
   @Test
   public void testMeshTerrainWalking()
   {
      runMeshTerrainWalking(false);
   }

   @Tag("humanoid-flat-ground-bullet")
   @Test
   public void testFlatGroundWalkingBullet()
   {
      runFlatGroundWalking(true);
   }

   public void runMeshTerrainWalking(boolean useBulletPhysicsEngine)
   {
      boolean doPelvisWarmup = doPelvisWarmup();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());
      simulationTestingParameters.setKeepSCSUp(true);

      MeshTerrainEnvironment meshTerrainEnvironment = new MeshTerrainEnvironment(MeshTerrainObjectFactory.createWorkPlatformObject(), MeshTerrainObjectFactory.createFlatGround());
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             meshTerrainEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, getWalkingScriptParameters());
      simulationTestHelperFactory.getHighLevelHumanoidControllerFactory().createUserDesiredControllerCommandGenerator();
      if (useBulletPhysicsEngine)
      {
         robotModel.getHumanoidRobotKinematicsCollisionModel();
         simulationTestHelperFactory.setUseBulletPhysicsEngine(useBulletPhysicsEngine);
      }
      physicsEngineName = useBulletPhysicsEngine ? "Bullet Physics Engine: " : "SCS2 Physics Engine: ";
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      if (CHECK_ICP_CONTINUITY)
         simulationTestHelper.addDesiredICPContinuityAssertion(3.0 * robotModel.getControllerDT());

      simulateAndAssertGoodWalking(simulationTestHelper, doPelvisWarmup);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

   }
   public void runFlatGroundWalking(boolean useBulletPhysicsEngine)
   {
      boolean doPelvisWarmup = doPelvisWarmup();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, getWalkingScriptParameters());
      simulationTestHelperFactory.getHighLevelHumanoidControllerFactory().createUserDesiredControllerCommandGenerator();
      if (useBulletPhysicsEngine)
      {
         robotModel.getHumanoidRobotKinematicsCollisionModel();
         simulationTestHelperFactory.setUseBulletPhysicsEngine(useBulletPhysicsEngine);
      }
      physicsEngineName = useBulletPhysicsEngine ? "Bullet Physics Engine: " : "SCS2 Physics Engine: ";
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      if (CHECK_ICP_CONTINUITY)
         simulationTestHelper.addDesiredICPContinuityAssertion(3.0 * robotModel.getControllerDT());

      simulateAndAssertGoodWalking(simulationTestHelper, doPelvisWarmup);

      //      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      //         simulationTestHelper.checkNothingChanged();

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testReset()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, getWalkingScriptParameters());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ((YoBoolean) simulationTestHelper.findVariable("walkCSG")).set(true);
      for (int i = 0; i < 10; i++)
      {
         Assert.assertTrue(simulationTestHelper.simulateNow(1.0));
         simulationTestHelper.resetRobot(false);
      }
   }

   private void simulateAndAssertGoodWalking(SCS2AvatarTestingSimulation simulationTestHelper, boolean doPelvisYawWarmup)
   {
      YoBoolean walk = (YoBoolean) simulationTestHelper.findVariable("walkCSG");
      YoDouble comError = (YoDouble) simulationTestHelper.findVariable("positionError_comHeight");
      if (comError == null)
      {
         comError = (YoDouble) simulationTestHelper.findVariable("pelvisErrorPositionZ");
      }
      YoBoolean userUpdateDesiredPelvisPose = (YoBoolean) simulationTestHelper.findVariable("userUpdateDesiredPelvisPose");
      YoBoolean userDoPelvisPose = (YoBoolean) simulationTestHelper.findVariable("userDoPelvisPose");
      YoDouble userDesiredPelvisPoseYaw = (YoDouble) simulationTestHelper.findVariable("userDesiredPelvisPoseYaw");
      YoDouble userDesiredPelvisPoseTrajectoryTime = (YoDouble) simulationTestHelper.findVariable("userDesiredPelvisPoseTrajectoryTime");
      YoDouble icpErrorX = (YoDouble) simulationTestHelper.findVariable("icpErrorX");
      YoDouble icpErrorY = (YoDouble) simulationTestHelper.findVariable("icpErrorY");

      YoDouble controllerICPErrorX = (YoDouble) simulationTestHelper.findVariable("controllerICPErrorX");
      YoDouble controllerICPErrorY = (YoDouble) simulationTestHelper.findVariable("controllerICPErrorY");

      assertTrue(simulationTestHelper.simulateNow(standingTimeDuration), "Simulation has failed");

      walk.set(false);

      if (doPelvisYawWarmup)
      {
         userDesiredPelvisPoseTrajectoryTime.set(0.0);
         userUpdateDesiredPelvisPose.set(true);
         assertTrue(simulationTestHelper.simulateNow(0.1), "Simulation has failed");

         double startingYaw = userDesiredPelvisPoseYaw.getDoubleValue();
         userDesiredPelvisPoseYaw.set(startingYaw + Math.PI / 4.0);
         userDoPelvisPose.set(true);

         assertTrue(simulationTestHelper.simulateNow(yawingTimeDuration), "Simulation has failed");

         double icpError;
         if (icpErrorX != null && icpErrorY != null)
            icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         else
            icpError = Math.sqrt(controllerICPErrorX.getDoubleValue() * controllerICPErrorX.getDoubleValue()
                                 + controllerICPErrorY.getDoubleValue() * controllerICPErrorY.getDoubleValue());
         assertTrue(icpError < 0.005, physicsEngineName + "icsError < 0.005 for startingYaw + pi/4.0 test");

         userDesiredPelvisPoseYaw.set(startingYaw);
         userDoPelvisPose.set(true);
         assertTrue(simulationTestHelper.simulateNow(yawingTimeDuration + 0.3), "Simulation has failed");

         if (icpErrorX != null && icpErrorY != null)
            icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         else
            icpError = Math.sqrt(controllerICPErrorX.getDoubleValue() * controllerICPErrorX.getDoubleValue()
                                 + controllerICPErrorY.getDoubleValue() * controllerICPErrorY.getDoubleValue());
         assertTrue(icpError < 0.005, physicsEngineName + "icsError < 0.005 for startingYaw test");
      }

      walk.set(true);

      double timeIncrement = 1.0;

      while (simulationTestHelper.getSimulationTime() - standingTimeDuration < defaultWalkingTimeDuration)
      {
         assertTrue(simulationTestHelper.simulateNow(timeIncrement), "Simulation has failed");
         if (Math.abs(comError.getDoubleValue()) > 0.06)
            fail(physicsEngineName + "Math.abs(comError.getDoubleValue()) > 0.06: " + comError.getDoubleValue() + " at t = "
                 + simulationTestHelper.getSimulationTime());
      }
   }

   //TODO: Get rid of the stuff below and use a test helper.....

   @AfterEach
   public void destroyOtherStuff()
   {

      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
         avatarSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }
   }

   private AvatarSimulation avatarSimulation;
   private RobotVisualizer robotVisualizer;

   protected void setupAndTestFlatGroundSimulationTrackTwice(DRCRobotModel robotModel)
   {
      SCS2AvatarTestingSimulation simulationTestHelperOne = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel);
      SCS2AvatarTestingSimulation simulationTestHelperTwo = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel);

      double walkingTimeDuration = 20.0;
      SCS2RunsSameWayTwiceVerifier verifier = new SCS2RunsSameWayTwiceVerifier(simulationTestHelperOne,
                                                                               simulationTestHelperTwo,
                                                                               standingTimeDuration,
                                                                               walkingTimeDuration);

      checkSimulationRunsSameWayTwice(verifier);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private SCS2AvatarTestingSimulation setupFlatGroundSimulationTrackForSameWayTwiceVerifier(DRCRobotModel robotModel)
   {
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, getWalkingScriptParameters());
      SCS2AvatarTestingSimulation simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();

      return simulationTestHelper;
   }

   private void checkSimulationRunsSameWayTwice(SCS2RunsSameWayTwiceVerifier verifier)
   {
      ArrayList<String> stringsToIgnore = new ArrayList<String>();
      stringsToIgnore.add("nano");
      stringsToIgnore.add("milli");
      stringsToIgnore.add("Timer");
      stringsToIgnore.add("actualControl");
      stringsToIgnore.add("actualEstimator");
      stringsToIgnore.add("totalDelay");
      stringsToIgnore.add("Time");

      double maxPercentDifference = 0.000001;
      assertTrue(verifier.verifySimRunsSameWayTwice(maxPercentDifference, stringsToIgnore), "Simulation did not run same way twice!");
   }

   public SimulationTestingParameters getSimulationTestingParameters()
   {
      return simulationTestingParameters;
   }

   public HeadingAndVelocityEvaluationScriptParameters getWalkingScriptParameters()
   {
      return null;
   }
}
