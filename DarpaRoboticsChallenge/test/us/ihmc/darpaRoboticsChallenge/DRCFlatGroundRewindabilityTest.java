package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.fail;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

@SuppressWarnings("deprecation")
public abstract class DRCFlatGroundRewindabilityTest implements MultiRobotTestInterface
{
   private static final boolean SHOW_GUI = false;
   private static final double totalTimeToTest = 10.0;
   private static final double timeToTickAhead = 1.5;
   private static final double timePerTick = 0.01;

   @Before
   public void setUp() throws Exception
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   
   @Ignore("Need to someday get rewindability working again. It will take using single threading probably.")
	@AverageDuration
	@Test(timeout=300000)
   public void testCanRewindAndGoForward() throws UnreasonableAccelerationException
   {
      BambooTools.reportTestStartedMessage();

      int numberOfSteps = 100;

      SimulationConstructionSet scs = setupScs();

      for (int i = 0; i < numberOfSteps; i++)
      {
         scs.simulateOneRecordStepNow();
         scs.simulateOneRecordStepNow();
         scs.stepBackwardNow();
      }

      scs.closeAndDispose();

      BambooTools.reportTestFinishedMessage();
   }

   @Ignore("Need to someday get rewindability working again. It will take using single threading probably.")
	@AverageDuration
	@Test(timeout=300000)
   public void testRewindability() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      int numTicksToTest = (int) Math.round(totalTimeToTest / timePerTick);
      if (numTicksToTest < 1)
         numTicksToTest = 1;

      int numTicksToSimulateAhead = (int) Math.round(timeToTickAhead / timePerTick);
      if (numTicksToSimulateAhead < 1)
         numTicksToSimulateAhead = 1;

      SimulationConstructionSet scs1 = setupScs();    // createTheSimulation(ticksForDataBuffer);
      SimulationConstructionSet scs2 = setupScs();    // createTheSimulation(ticksForDataBuffer);
      
      BlockingSimulationRunner blockingSimulationRunner1 = new BlockingSimulationRunner(scs1, 1000.0);
      BlockingSimulationRunner blockingSimulationRunner2 = new BlockingSimulationRunner(scs2, 1000.0);
      
      BooleanYoVariable walk1 = (BooleanYoVariable) scs1.getVariable("walk");
      BooleanYoVariable walk2 = (BooleanYoVariable) scs2.getVariable("walk");
      
//      double standingTimeDuration = 1.0;
//      double walkingTimeDuration = 1.0;
//      initiateMotion(standingTimeDuration, walkingTimeDuration, blockingSimulationRunner1, walk1);      
//      initiateMotion(standingTimeDuration, walkingTimeDuration, blockingSimulationRunner2, walk2);

      walk1.set(true);
      walk2.set(true);
      
      ArrayList<String> exceptions = new ArrayList<String>();
      exceptions.add("gc_");
      exceptions.add("toolFrame");
      exceptions.add("ef_");
      exceptions.add("kp_");
      exceptions.add("TimeNano");
      exceptions.add("DurationMilli");
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);

      // TODO velocityMagnitudeInHeading usually differs by 0.00125
      double maxDifferenceAllowed = 1e-7;
      ArrayList<VariableDifference> variableDifferences;getClass();
      variableDifferences = checker.checkRewindabilityWithSimpleMethod(numTicksToTest, maxDifferenceAllowed);
      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         if (SHOW_GUI)
            sleepForever();
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }

      // sleepForever();

      scs1.closeAndDispose();
      scs2.closeAndDispose();

      BambooTools.reportTestFinishedMessage();
   }

   private SimulationConstructionSet setupScs()
   {
      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      GroundProfile3D groundProfile = new FlatGroundProfile();
      DRCRobotModel robotModel = getRobotModel();

      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();

      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setupCameraForUnitTest(scs);
      
      return scs;
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

   private DRCGuiInitialSetup createGUIInitialSetup()
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      guiInitialSetup.setCreateGUI(SHOW_GUI);

      return guiInitialSetup;
   }

   private void initiateMotion(double standingTimeDuration, double walkingTimeDuration, BlockingSimulationRunner runner, BooleanYoVariable walk)
           throws SimulationExceededMaximumTimeException
   {
      walk.set(false);
      runner.simulateAndBlock(standingTimeDuration);
      walk.set(true);
//      runner.simulateAndBlock(walkingTimeDuration);
   }

   private void sleepForever()
   {
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }

      }
   }
}
