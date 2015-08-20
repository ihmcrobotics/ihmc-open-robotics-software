package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;
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
import us.ihmc.stateEstimation.humanoid.DRCSimulatedSensorNoiseParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.SdfLoader.visualizer.RobotVisualizer;

public abstract class DRCFlatGroundWalkingWithIMUDriftTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   

   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;

   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = BambooTools.isEveryCommitBuild() ? 45.0 : 90.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static final boolean cheatWithGroundHeightAtForFootstep = false;
   private static final boolean drawGroundProfile = false;

 
   
   @After
   public void tearDown()
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

   protected void setupAndTestFlatGroundSimulationTrack(DRCRobotModel robotModel, String runName) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      DRCFlatGroundWalkingTrack track = setupFlatGroundSimulationTrack(robotModel);

      simulateAndAssertGoodWalking(track, runName);
   }
   
   protected void simulateAndAssertGoodWalking(DRCFlatGroundWalkingTrack track, String runName) throws SimulationExceededMaximumTimeException, ControllerFailureException
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

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      DoubleYoVariable comError = (DoubleYoVariable) scs.getVariable("positionError_comHeight");

      initiateMotion(scs, standingTimeDuration, blockingSimulationRunner);

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

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
         checkNothingChanged(nothingChangedVerifier);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }
 
   private void initiateMotion(SimulationConstructionSet scs, double standingTimeDuration, BlockingSimulationRunner runner)
           throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
      walk.set(false);
      runner.simulateAndBlock(standingTimeDuration);
//      walk.set(true);
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSMovies())
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 1);
      }
   }

   protected DRCFlatGroundWalkingTrack setupFlatGroundSimulationTrack(DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();
      GroundProfile3D groundProfile = new FlatGroundProfile();
      
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(drawGroundProfile);
      scsInitialSetup.setSimulatedSensorNoiseParameters(DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise());
      
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
