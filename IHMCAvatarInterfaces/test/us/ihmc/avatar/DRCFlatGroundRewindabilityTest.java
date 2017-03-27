package us.ihmc.avatar;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCFlatGroundRewindabilityTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;

   @Before
   public void setUp() throws Exception
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      simulationTestingParameters.setRunMultiThreaded(false);
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 9.2)
   @Test(timeout = 46000)
   public void testCanRewindAndGoForward() throws UnreasonableAccelerationException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      int numberOfSteps = 100;
      SimulationConstructionSet scs = setupScs();
      scs.simulateOneRecordStepNow();
      scs.simulateOneRecordStepNow();

      for (int i = 0; i < numberOfSteps; i++)
      {
         scs.simulateOneRecordStepNow();
         scs.simulateOneRecordStepNow();
         scs.stepBackwardNow();
      }

      scs.closeAndDispose();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 43.4)
   @Test(timeout = 220000)
   public void testRunsTheSameWayTwice() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      SimulationConstructionSet scs1 = setupScs();
      SimulationConstructionSet scs2 = setupScs();
      ArrayList<String> exceptions = DRCSimulationTestHelper.createVariableNamesStringsToIgnore();
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      BlockingSimulationRunner blockingSimulationRunner1 = new BlockingSimulationRunner(scs1, 1000.0);
      BlockingSimulationRunner blockingSimulationRunner2 = new BlockingSimulationRunner(scs2, 1000.0);
      BooleanYoVariable walk1 = (BooleanYoVariable) scs1.getVariable("walk");
      BooleanYoVariable walk2 = (BooleanYoVariable) scs2.getVariable("walk");
      double standingTimeDuration = 1.0;
      double walkingTimeDuration = 4.0;
      initiateWalkingMotion(standingTimeDuration, walkingTimeDuration, blockingSimulationRunner1, walk1);
      initiateWalkingMotion(standingTimeDuration, walkingTimeDuration, blockingSimulationRunner2, walk2);
      ArrayList<VariableDifference> variableDifferences = checker.verifySimulationsAreSameToStart();
      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         if (simulationTestingParameters.getKeepSCSUp())
            ThreadTools.sleepForever();
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }

      scs1.closeAndDispose();
      scs2.closeAndDispose();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // "Bamboo link: https://bamboo.ihmc.us/browse/RC-FASTLOOP-ATLASAFAST/test/case/115277833")
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 2400000)
   public void testRewindabilityWithSimpleFastMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      double totalTimeBeforeWalking = 2.0;
      double totalTimeAfterWalking = 4.0;
      SimulationConstructionSet scs1 = setupScs();
      SimulationConstructionSet scs2 = setupScs();
      double timePerTick = scs1.getTimePerRecordTick();
      int numberTicksBeforeWalking = (int) Math.round(totalTimeBeforeWalking / timePerTick);

      // Get past the initialization hump.
      int numTicksToStartComparingAt = 2;
      int numberTicksAfterWalking = (int) Math.round(totalTimeAfterWalking / timePerTick);
      ArrayList<String> exceptions = DRCSimulationTestHelper.createVariableNamesStringsToIgnore();
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      double maxDifferenceAllowed = 1e-14;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<VariableDifference>();
      int numTicksToSimulateAhead = 1;
      checker.checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt, numberTicksBeforeWalking, numTicksToSimulateAhead, maxDifferenceAllowed,
              variableDifferences);

      // checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksBeforeWalking, maxDifferenceAllowed, variableDifferences);
      checkForVariableDifferences(variableDifferences);
      BooleanYoVariable walk1 = (BooleanYoVariable) scs1.getVariable("walk");
      BooleanYoVariable walk2 = (BooleanYoVariable) scs2.getVariable("walk");
      walk1.set(true);
      walk2.set(true);
      numTicksToSimulateAhead = 1;

      // Must be greater than zero since the footstep provider stuff is not rewindable...
      numTicksToStartComparingAt = 1;
      checker.checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt, numberTicksAfterWalking, numTicksToSimulateAhead, maxDifferenceAllowed,
              variableDifferences);

      // checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksAfterWalking, maxDifferenceAllowed, variableDifferences);
      checkForVariableDifferences(variableDifferences);

      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      scs1.closeAndDispose();
      scs2.closeAndDispose();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // This takes a long time. Use it for debugging where the broken changes were made when the tests above fail.
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 2400000)
   public void testRewindabilityWithSlowerMoreExtensiveMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      double totalTimeBeforeWalking = 2.0;
      double totalTimeAfterWalking = 4.0;
      SimulationConstructionSet scs1 = setupScs();
      SimulationConstructionSet scs2 = setupScs();
      double timePerTick = scs1.getTimePerRecordTick();
      int numberTicksBeforeWalking = (int) Math.round(totalTimeBeforeWalking / timePerTick);

      // Get past the initialization hump.
      int numTicksToStartComparingAt = 2;
      int numberTicksAfterWalking = (int) Math.round(totalTimeAfterWalking / timePerTick);
      ArrayList<String> exceptions = DRCSimulationTestHelper.createVariableNamesStringsToIgnore();
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      double maxDifferenceAllowed = 1e-14;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<VariableDifference>();

      // checker.checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numberTicksBeforeWalking, maxDifferenceAllowed, variableDifferences);
      checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksBeforeWalking,
              maxDifferenceAllowed, variableDifferences);
      checkForVariableDifferences(variableDifferences);
      BooleanYoVariable walk1 = (BooleanYoVariable) scs1.getVariable("walk");
      BooleanYoVariable walk2 = (BooleanYoVariable) scs2.getVariable("walk");
      walk1.set(true);
      walk2.set(true);

      // Must be greater than zero since the footstep provider stuff is not rewindable...
      numTicksToStartComparingAt = 1;

      // checker.checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numberTicksAfterWalking, maxDifferenceAllowed, variableDifferences);
      checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksAfterWalking, maxDifferenceAllowed,
              variableDifferences);
      checkForVariableDifferences(variableDifferences);

      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      scs1.closeAndDispose();
      scs2.closeAndDispose();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void checkForVariableDifferences(ArrayList<VariableDifference> variableDifferences)
   {
      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         if (simulationTestingParameters.getKeepSCSUp())
            ThreadTools.sleepForever();
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }
   }

   private SimulationConstructionSet setupScs()
   {
      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;
      GroundProfile3D groundProfile = new FlatGroundProfile();
      DRCRobotModel robotModel = getRobotModel();
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setRunMultiThreaded(false);
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
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);

      return guiInitialSetup;
   }

   private void initiateWalkingMotion(double standingTimeDuration, double walkingTimeDuration, BlockingSimulationRunner runner, BooleanYoVariable walk)
           throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      walk.set(false);
      runner.simulateAndBlock(standingTimeDuration);
      walk.set(true);
      runner.simulateAndBlock(walkingTimeDuration);
   }
}
