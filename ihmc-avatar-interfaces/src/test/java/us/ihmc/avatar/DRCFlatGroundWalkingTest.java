package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.simulationTesting.SimulationRunsSameWayTwiceVerifier;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

@Tag("video")
public abstract class DRCFlatGroundWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   /**
    * TODO Need to implement a specific test for that.
    * As the footstep generator for flat ground walking keeps changing the upcoming footsteps on the fly, the ICP planner ends up creating discontinuities.
    * But this is an expected behavior.
    */
   private static final boolean CHECK_ICP_CONTINUITY = false;

   private static final double yawingTimeDuration = 0.5;
   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = BambooTools.isEveryCommitBuild() ? 45.0 : 90.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static final boolean cheatWithGroundHeightAtForFootstep = false;
   private static final boolean drawGroundProfile = false;

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
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
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

   @Test
   public void testFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      runFlatGroundWalking();
   }

   public void runFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      DRCRobotModel robotModel = getRobotModel();
      boolean doPelvisWarmup = doPelvisWarmup();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(useVelocityAndHeadingScript);
      drcSimulationTestHelper.setCheatWithGroundHeightAtFootstep(cheatWithGroundHeightAtForFootstep);
      drcSimulationTestHelper.setWalkingScriptParameters(getWalkingScriptParameters());
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForUnitTest(scs);
      simulateAndAssertGoodWalking(drcSimulationTestHelper, doPelvisWarmup);

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
         drcSimulationTestHelper.checkNothingChanged();
      if (CHECK_ICP_CONTINUITY)
         verifyDesiredICPIsContinous(scs);

      createVideo(scs);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testReset() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(useVelocityAndHeadingScript);
      drcSimulationTestHelper.setCheatWithGroundHeightAtFootstep(cheatWithGroundHeightAtForFootstep);
      drcSimulationTestHelper.setWalkingScriptParameters(getWalkingScriptParameters());
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "Reset");

      ((YoBoolean) drcSimulationTestHelper.getYoVariable("walkCSG")).set(true);
      for (int i = 0; i < 10; i++)
      {
         Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
         drcSimulationTestHelper.getAvatarSimulation().resetRobot(false);
      }
   }

   private void simulateAndAssertGoodWalking(DRCSimulationTestHelper drcSimulationTestHelper, boolean doPelvisYawWarmup)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      YoBoolean walk = (YoBoolean) scs.findVariable("walkCSG");
      YoDouble comError = (YoDouble) scs.findVariable("positionError_comHeight");
      if (comError == null)
      {
         comError = (YoDouble) scs.findVariable("pelvisErrorPositionZ");
      }
      YoBoolean userUpdateDesiredPelvisPose = (YoBoolean) scs.findVariable("userUpdateDesiredPelvisPose");
      YoBoolean userDoPelvisPose = (YoBoolean) scs.findVariable("userDoPelvisPose");
      YoDouble userDesiredPelvisPoseYaw = (YoDouble) scs.findVariable("userDesiredPelvisPoseYaw");
      YoDouble userDesiredPelvisPoseTrajectoryTime = (YoDouble) scs.findVariable("userDesiredPelvisPoseTrajectoryTime");
      YoDouble icpErrorX = (YoDouble) scs.findVariable("icpErrorX");
      YoDouble icpErrorY = (YoDouble) scs.findVariable("icpErrorY");

      YoDouble controllerICPErrorX = (YoDouble) scs.findVariable("controllerICPErrorX");
      YoDouble controllerICPErrorY = (YoDouble) scs.findVariable("controllerICPErrorY");

      drcSimulationTestHelper.simulateAndBlock(standingTimeDuration);

      walk.set(false);

      if (doPelvisYawWarmup)
      {
         userDesiredPelvisPoseTrajectoryTime.set(0.0);
         userUpdateDesiredPelvisPose.set(true);
         drcSimulationTestHelper.simulateAndBlock(0.1);

         double startingYaw = userDesiredPelvisPoseYaw.getDoubleValue();
         userDesiredPelvisPoseYaw.set(startingYaw + Math.PI/4.0);
         userDoPelvisPose.set(true);

         drcSimulationTestHelper.simulateAndBlock(yawingTimeDuration);

         double icpError;
         if (icpErrorX != null && icpErrorY != null)
            icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         else
            icpError = Math.sqrt(controllerICPErrorX.getDoubleValue() * controllerICPErrorX.getDoubleValue() + controllerICPErrorY.getDoubleValue() * controllerICPErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);

         userDesiredPelvisPoseYaw.set(startingYaw);
         userDoPelvisPose.set(true);
         drcSimulationTestHelper.simulateAndBlock(yawingTimeDuration + 0.3);

         if (icpErrorX != null && icpErrorY != null)
            icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         else
            icpError = Math.sqrt(controllerICPErrorX.getDoubleValue() * controllerICPErrorX.getDoubleValue() + controllerICPErrorY.getDoubleValue() * controllerICPErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);
      }

      walk.set(true);

      double timeIncrement = 1.0;

      while (scs.getTime() - standingTimeDuration < defaultWalkingTimeDuration)
      {
         drcSimulationTestHelper.simulateAndBlock(timeIncrement);
         if (Math.abs(comError.getDoubleValue()) > 0.06)
            fail("Math.abs(comError.getDoubleValue()) > 0.06: " + comError.getDoubleValue() + " at t = " + scs.getTime());
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

   protected void setupAndTestFlatGroundSimulationTrackTwice(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      simulateAndAssertSimRunsSameWayTwice(robotModel);
   }

   private void simulateAndAssertSimRunsSameWayTwice(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      SimulationConstructionSet scsOne = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel).getSimulationConstructionSet();
      SimulationConstructionSet scsTwo = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel).getSimulationConstructionSet();

      double walkingTimeDuration = 20.0;
      SimulationRunsSameWayTwiceVerifier verifier = new SimulationRunsSameWayTwiceVerifier(scsOne, scsTwo, standingTimeDuration, walkingTimeDuration);

      checkSimulationRunsSameWayTwice(verifier);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void verifyDesiredICPIsContinous(SimulationConstructionSet scs)
   {
      YoDouble desiredICPX = (YoDouble) scs.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) scs.findVariable("desiredICPY");
      YoDouble t = (YoDouble) scs.findVariable("t");

      scs.gotoInPointNow();
      while(Math.abs(desiredICPX.getDoubleValue()) < 1e-4)
      {
         scs.tickAndReadFromBuffer(1);
      }
      scs.setInPoint();

      scs.cropBuffer();
      double[] desiredICPXData = scs.getDataBuffer().getEntry(desiredICPX).getBuffer();
      double[] desiredICPYData = scs.getDataBuffer().getEntry(desiredICPY).getBuffer();


      double[] tValues = scs.getDataBuffer().getEntry(t).getBuffer();
      double dt = tValues[1] - tValues[0];

      // Setting max velocity of desired ICP to 3.0.
      // This will need to increase once we start walking faster.
      // Then we'll need more clever icp continuity checks.

      double maxChangePerTick = 3.0 * dt;

      boolean icpXIsContinuous = ArrayTools.isContinuous(desiredICPXData, maxChangePerTick);
      boolean icpYIsContinuous = ArrayTools.isContinuous(desiredICPYData, maxChangePerTick);

      if (!icpXIsContinuous || !icpYIsContinuous)
      {
         double xMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredICPXData);
         int indexOfXMaxChange = ArrayTools.getIndexOfMaximumAbsoluteChangeBetweenTicks(desiredICPXData);
         double yMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredICPYData);
         int indexOfYMaxChange = ArrayTools.getIndexOfMaximumAbsoluteChangeBetweenTicks(desiredICPYData);

         System.err.println("Desired ICP xMaxChange = " + xMaxChange + ", at t = " + tValues[indexOfXMaxChange]);
         System.err.println("Desired ICP yMaxChange = " + yMaxChange + ", at t = " + tValues[indexOfYMaxChange]);

         fail("Desired ICP is not continuous!");
      }
   }


   private void createVideo(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 2);
      }
      else
      {
         LogTools.info("Skipping video generation.");
      }
   }

   private DRCFlatGroundWalkingTrack setupFlatGroundSimulationTrackForSameWayTwiceVerifier(DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();

      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(drawGroundProfile);

      if (cheatWithGroundHeightAtForFootstep)
         scsInitialSetup.setInitializeEstimatorToActual(true);

      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup,
            scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setupCameraForUnitTest(scs);

      return drcFlatGroundWalkingTrack;
   }

   private void checkSimulationRunsSameWayTwice(SimulationRunsSameWayTwiceVerifier verifier) throws SimulationExceededMaximumTimeException, ControllerFailureException
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
      assertTrue("Simulation did not run same way twice!", verifier.verifySimRunsSameWayTwice(maxPercentDifference, stringsToIgnore));
   }

   private DRCGuiInitialSetup createGUIInitialSetup()
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);
      return guiInitialSetup;
   }

   private void setupCameraForUnitTest(SimulationConstructionSet scs)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.4, 1.1);
      cameraConfiguration.setCameraPosition(-0.15, 10.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, true, false);
      cameraConfiguration.setCameraDolly(true, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
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
