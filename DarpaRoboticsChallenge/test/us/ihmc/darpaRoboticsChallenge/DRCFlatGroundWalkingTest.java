package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationRunsSameWayTwiceVerifier;
import us.ihmc.tools.ArrayTools;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.SdfLoader.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCFlatGroundWalkingTest implements MultiRobotTestInterface
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }
      
      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   //TODO: Get rid of the stuff below and use a test helper.....
   
   @After
   public void destroyOtherStuff()
   {
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
   }
   

   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;

   private static final double yawingTimeDuration = 0.1;
   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = BambooTools.isEveryCommitBuild() ? 45.0 : 90.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static final boolean cheatWithGroundHeightAtForFootstep = false;
   private static final boolean drawGroundProfile = false;

   protected void setupAndTestFlatGroundSimulationTrack(DRCRobotModel robotModel, String runName, boolean doPelvisYawWarmup) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      DRCFlatGroundWalkingTrack track = setupFlatGroundSimulationTrack(robotModel);

      simulateAndAssertGoodWalking(track, runName, doPelvisYawWarmup);
   }

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

      BambooTools.reportTestFinishedMessage();
   }
   
   private void simulateAndAssertGoodWalking(DRCFlatGroundWalkingTrack track, String runName, boolean doPelvisYawWarmup) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      NothingChangedVerifier nothingChangedVerifier = null;
      double walkingTimeDuration;
      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         nothingChangedVerifier = new NothingChangedVerifier(runName, scs);
         walkingTimeDuration = 7.0;
      }
      else
         walkingTimeDuration = defaultWalkingTimeDuration;

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      track.attachControllerFailureListener(blockingSimulationRunner.createControllerFailureListener());

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
//    DoubleYoVariable desiredHeading = (DoubleYoVariable) scs.getVariable("desiredHeading");
//    DoubleYoVariable pelvisYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
//    DoubleYoVariable centerOfMassHeight = (DoubleYoVariable) scs.getVariable("ProcessedSensors.comPositionz");
      DoubleYoVariable comError = (DoubleYoVariable) scs.getVariable("positionError_comHeight");

      DoubleYoVariable userDesiredPelvisYaw = (DoubleYoVariable) scs.getVariable("userDesiredPelvisYaw");
      DoubleYoVariable userPelvisTrajectoryTime = (DoubleYoVariable) scs.getVariable("userDesiredPelvisTrajectoryTime");
      DoubleYoVariable icpErrorX = (DoubleYoVariable) scs.getVariable("icpErrorX");
      DoubleYoVariable icpErrorY = (DoubleYoVariable) scs.getVariable("icpErrorY");

      blockingSimulationRunner.simulateAndBlock(standingTimeDuration);

      walk.set(false);

      if (doPelvisYawWarmup)
      {
         userPelvisTrajectoryTime.set(0.0);
         userDesiredPelvisYaw.set(Math.PI/4.0);

         blockingSimulationRunner.simulateAndBlock(yawingTimeDuration);

         double icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);

         userDesiredPelvisYaw.set(0.0);
         blockingSimulationRunner.simulateAndBlock(yawingTimeDuration + 0.3);

         icpError = Math.sqrt(icpErrorX.getDoubleValue() * icpErrorX.getDoubleValue() + icpErrorY.getDoubleValue() * icpErrorY.getDoubleValue());
         assertTrue(icpError < 0.005);
      }

      walk.set(true);

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
      
      verifyDesiredICPIsContinous(scs);
      
      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
         checkNothingChanged(nothingChangedVerifier);

      createVideo(scs);
      BambooTools.reportTestFinishedMessage();
   }

   private void verifyDesiredICPIsContinous(SimulationConstructionSet scs)
   {
      DoubleYoVariable desiredICPX = (DoubleYoVariable) scs.getVariable("desiredICPX");
      DoubleYoVariable desiredICPY = (DoubleYoVariable) scs.getVariable("desiredICPY");
      DoubleYoVariable t = (DoubleYoVariable) scs.getVariable("t");
      
      scs.gotoInPointNow();
      while(Math.abs(desiredICPX.getDoubleValue()) < 1e-4)
      {
         scs.tick(1);
      }
      scs.setInPoint();
      
      scs.cropBuffer();
      double[] desiredICPXData = scs.getDataBuffer().getEntry(desiredICPX).getData();
      double[] desiredICPYData = scs.getDataBuffer().getEntry(desiredICPY).getData();

      
      double[] tValues = scs.getDataBuffer().getEntry(t).getData();
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
         double yMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredICPYData);
         
         System.err.println("Desired ICP xMaxChange = " + xMaxChange);
         System.err.println("Desired ICP yMaxChange = " + yMaxChange);

         fail("Desired ICP is not continuous!");
      }
   }
   

   private void createVideo(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 3);
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

      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      
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

      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

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
}
