package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import com.yobotics.simulationconstructionset.util.simulationTesting.SimulationRunsSameWayTwiceVerifier;
import org.junit.After;
import org.junit.Before;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import com.yobotics.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;

@SuppressWarnings("deprecation")
public abstract class DRCFlatGroundWalkingTest implements MultiRobotTestInterface
{
   private static final boolean ALWAYS_SHOW_GUI = false;
   public static final boolean KEEP_SCS_UP = false;

   private static final boolean CREATE_MOVIE = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();

   private static final boolean SHOW_GUI = ALWAYS_SHOW_GUI || checkNothingChanged || CREATE_MOVIE;

   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;

   private static final double yawingTimeDuration = 0.1;
   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = BambooTools.isEveryCommitBuild() ? 45.0 : 90.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static final boolean cheatWithGroundHeightAtForFootstep = false;
   private static final boolean drawGroundProfile = false;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      if (drcSimulation != null)
      {
         drcSimulation.dispose();
         drcSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   protected void setupAndTestFlatGroundSimulationTrack(DRCRobotModel robotModel, String runName, boolean doPelvisYawWarmup) throws SimulationExceededMaximumTimeException
   {
      DRCFlatGroundWalkingTrack track = setupFlatGroundSimulationTrack(robotModel);

      simulateAndAssertGoodWalking(track, runName, doPelvisYawWarmup);
   }

   protected void setupAndTestFlatGroundSimulationTrackTwice(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException
   {
      simulateAndAssertSimRunsSameWayTwice(robotModel);
   }

   private void simulateAndAssertSimRunsSameWayTwice(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException
   {
      SimulationConstructionSet scsOne = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel).getSimulationConstructionSet();
      SimulationConstructionSet scsTwo = setupFlatGroundSimulationTrackForSameWayTwiceVerifier(robotModel).getSimulationConstructionSet();

      double walkingTimeDuration = 20.0;
      SimulationRunsSameWayTwiceVerifier verifier = new SimulationRunsSameWayTwiceVerifier(scsOne, scsTwo, standingTimeDuration, walkingTimeDuration);

      checkSimulationRunsSameWayTwice(verifier);

      BambooTools.reportTestFinishedMessage();
   }
   
   private void simulateAndAssertGoodWalking(DRCFlatGroundWalkingTrack track, String runName, boolean doPelvisYawWarmup) throws SimulationExceededMaximumTimeException
   {
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      NothingChangedVerifier nothingChangedVerifier = null;
      double walkingTimeDuration;
      if (checkNothingChanged)
      {
         nothingChangedVerifier = new NothingChangedVerifier(runName, scs);
         walkingTimeDuration = 7.0;
      }
      else
         walkingTimeDuration = defaultWalkingTimeDuration;

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

//    DoubleYoVariable desiredHeading = (DoubleYoVariable) scs.getVariable("desiredHeading");
//    DoubleYoVariable pelvisYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
//    DoubleYoVariable centerOfMassHeight = (DoubleYoVariable) scs.getVariable("ProcessedSensors.comPositionz");
      DoubleYoVariable comError = (DoubleYoVariable) scs.getVariable("positionError_comHeight");

      DoubleYoVariable userDesiredPelvisYaw = (DoubleYoVariable) scs.getVariable("userDesiredPelvisYaw");
      BooleanYoVariable userSetDesiredPelvis = (BooleanYoVariable) scs.getVariable("userSetDesiredPelvis");
      DoubleYoVariable icpErrorX = (DoubleYoVariable) scs.getVariable("icpErrorX");
      DoubleYoVariable icpErrorY = (DoubleYoVariable) scs.getVariable("icpErrorY");

      initiateMotion(scs, standingTimeDuration, blockingSimulationRunner);

      if (doPelvisYawWarmup)
      {
         userSetDesiredPelvis.set(true);
         userDesiredPelvisYaw.add(Math.PI/4.0);

         initiateMotion(scs, yawingTimeDuration, blockingSimulationRunner);

         double icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);

         userSetDesiredPelvis.set(true);
         userDesiredPelvisYaw.sub(Math.PI/4.0);
         initiateMotion(scs, yawingTimeDuration, blockingSimulationRunner);

         userSetDesiredPelvis.set(false);

         icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);
      }

      double timeIncrement = 1.0;

      while (scs.getTime() - standingTimeDuration < walkingTimeDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);

         // TODO: Put test for heading back in here.
//       if (!MathTools.epsilonEquals(desiredHeading.getDoubleValue(), pelvisYaw.getDoubleValue(), epsilonHeading))
//       {
//          fail("Desired Heading too large of error: " + desiredHeading.getDoubleValue());
//       }

         //TODO: Reduce the error tolerance from 2.5 cm to under 1 cm after we change things so that we are truly 
         // controlling pelvis height, not CoM height.
         if (Math.abs(comError.getDoubleValue()) > 0.06)
         {
            fail("Math.abs(comError.getDoubleValue()) > 0.06: " + comError.getDoubleValue() + " at t = " + scs.getTime());
         }
      }

      if (checkNothingChanged)
         checkNothingChanged(nothingChangedVerifier);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }
 
   private void initiateMotion(SimulationConstructionSet scs, double standingTimeDuration, BlockingSimulationRunner runner)
           throws SimulationExceededMaximumTimeException
   {
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
      walk.set(false);
      runner.simulateAndBlock(standingTimeDuration);
      walk.set(true);
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (CREATE_MOVIE)
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 1);
      }
   }

   private DRCFlatGroundWalkingTrack setupFlatGroundSimulationTrack(DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();
      
      GroundProfile3D groundProfile = new FlatGroundProfile();
      
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(drawGroundProfile);
      
      if (cheatWithGroundHeightAtForFootstep)
         scsInitialSetup.setInitializeEstimatorToActual(true);

      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      
      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup,
                                                               scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setupCameraForUnitTest(scs);

      drcSimulation = drcFlatGroundWalkingTrack.getDrcSimulation();

      return drcFlatGroundWalkingTrack;
   }

   private DRCFlatGroundWalkingTrack setupFlatGroundSimulationTrackForSameWayTwiceVerifier(DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();

      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(drawGroundProfile);

      if (cheatWithGroundHeightAtForFootstep)
         scsInitialSetup.setInitializeEstimatorToActual(true);

      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup,
            scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setupCameraForUnitTest(scs);

      return drcFlatGroundWalkingTrack;
   }

   private void checkNothingChanged(NothingChangedVerifier nothingChangedVerifier)
   {
      ArrayList<String> stringsToIgnore = new ArrayList<String>();
      stringsToIgnore.add("nano");
      stringsToIgnore.add("milli");
      stringsToIgnore.add("Timer");

      boolean writeNewBaseFile = nothingChangedVerifier.getWriteNewBaseFile();

      double maxPercentDifference = 0.001;
      nothingChangedVerifier.verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
      assertFalse("Had to write new base file. On next run nothing should change", writeNewBaseFile);
   }

   private void checkSimulationRunsSameWayTwice(SimulationRunsSameWayTwiceVerifier verifier) throws SimulationExceededMaximumTimeException
   {
      ArrayList<String> stringsToIgnore = new ArrayList<String>();
      stringsToIgnore.add("nano");
      stringsToIgnore.add("milli");
      stringsToIgnore.add("Timer");
      stringsToIgnore.add("actualControl");
      stringsToIgnore.add("actualEstimator");
      stringsToIgnore.add("totalDelay");

      double maxPercentDifference = 0.000001;
      assertTrue("Simulation did not run same way twice!", verifier.verifySimRunsSameWayTwice(maxPercentDifference, stringsToIgnore));
   }

   private DRCGuiInitialSetup createGUIInitialSetup()
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      guiInitialSetup.setIsGuiShown(SHOW_GUI);

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
}
