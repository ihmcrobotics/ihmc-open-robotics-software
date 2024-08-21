package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2RewindabilityVerifier;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class DRCFlatGroundRewindabilityTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;

   @BeforeEach
   public void setUp() throws Exception
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void showMemoryUsageAfterTest()
   {
      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testCanRewindAndGoForward()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      int numberOfSteps = 100;
      SCS2AvatarTestingSimulation simulationTestHelper = setupSimulation();
      simulationTestHelper.start();
      simulationTestHelper.simulateOneBufferRecordPeriodNow();
      simulationTestHelper.simulateOneBufferRecordPeriodNow();

      for (int i = 0; i < numberOfSteps; i++)
      {
         simulationTestHelper.simulateOneBufferRecordPeriodNow();
         simulationTestHelper.simulateOneBufferRecordPeriodNow();
         simulationTestHelper.stepBufferIndexBackward();
      }

      simulationTestHelper.finishTest();
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testRunsTheSameWayTwice()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      SCS2AvatarTestingSimulation simulationTestHelper1 = setupSimulation();
      SCS2AvatarTestingSimulation simulationTestHelper2 = setupSimulation();
      simulationTestHelper1.start();
      simulationTestHelper2.start();
      List<String> exceptions = createVariableNamesStringsToIgnore();
      SCS2RewindabilityVerifier checker = new SCS2RewindabilityVerifier(simulationTestHelper1, simulationTestHelper2, exceptions);
      YoBoolean walk1 = (YoBoolean) simulationTestHelper1.findVariable("walkCSG");
      YoBoolean walk2 = (YoBoolean) simulationTestHelper2.findVariable("walkCSG");
      double standingTimeDuration = 1.0;
      double walkingTimeDuration = 4.0;
      initiateWalkingMotion(standingTimeDuration, walkingTimeDuration, simulationTestHelper1, walk1);
      initiateWalkingMotion(standingTimeDuration, walkingTimeDuration, simulationTestHelper2, walk2);
      List<VariableDifference> variableDifferences = checker.verifySimulationsAreSameToStart();
      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }

      simulationTestHelper1.finishTest();
      simulationTestHelper2.finishTest();
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Disabled
   @Test
   public void testRewindabilityWithSimpleFastMethod()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      double totalTimeBeforeWalking = 2.0;
      double totalTimeAfterWalking = 4.0;
      SCS2AvatarTestingSimulation simulationTestHelper1 = setupSimulation();
      SCS2AvatarTestingSimulation simulationTestHelper2 = setupSimulation();
      simulationTestHelper1.start();
      simulationTestHelper2.start();
      double timePerTick = simulationTestHelper1.getTimePerRecordTick();
      int numberTicksBeforeWalking = (int) Math.round(totalTimeBeforeWalking / timePerTick);

      // Get past the initialization hump.
      int numTicksToStartComparingAt = 2;
      int numberTicksAfterWalking = (int) Math.round(totalTimeAfterWalking / timePerTick);
      List<String> exceptions = createVariableNamesStringsToIgnore();
      SCS2RewindabilityVerifier checker = new SCS2RewindabilityVerifier(simulationTestHelper1, simulationTestHelper2, exceptions);
      double maxDifferenceAllowed = 1e-14;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<>();
      int numTicksToSimulateAhead = 1;
      checker.checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt,
                                                   numberTicksBeforeWalking,
                                                   numTicksToSimulateAhead,
                                                   maxDifferenceAllowed,
                                                   variableDifferences);

      // checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksBeforeWalking, maxDifferenceAllowed, variableDifferences);
      checkForVariableDifferences(variableDifferences);
      YoBoolean walk1 = (YoBoolean) simulationTestHelper1.findVariable("walkCSG");
      YoBoolean walk2 = (YoBoolean) simulationTestHelper2.findVariable("walkCSG");
      walk1.set(true);
      walk2.set(true);
      numTicksToSimulateAhead = 1;

      // Must be greater than zero since the footstep provider stuff is not rewindable...
      numTicksToStartComparingAt = 1;
      checker.checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt,
                                                   numberTicksAfterWalking,
                                                   numTicksToSimulateAhead,
                                                   maxDifferenceAllowed,
                                                   variableDifferences);

      // checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt, numberTicksAfterWalking, maxDifferenceAllowed, variableDifferences);
      checkForVariableDifferences(variableDifferences);

      simulationTestHelper1.finishTest();
      simulationTestHelper2.finishTest();
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Disabled
   @Test
   public void testRewindabilityWithSlowerMoreExtensiveMethod()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      double totalTimeBeforeWalking = 2.0;
      double totalTimeAfterWalking = 4.0;
      SCS2AvatarTestingSimulation simulationTestHelper1 = setupSimulation();
      SCS2AvatarTestingSimulation simulationTestHelper2 = setupSimulation();
      simulationTestHelper1.start();
      simulationTestHelper2.start();
      double timePerTick = simulationTestHelper1.getTimePerRecordTick();
      int numberTicksBeforeWalking = (int) Math.round(totalTimeBeforeWalking / timePerTick);

      // Get past the initialization hump.
      int numTicksToStartComparingAt = 2;
      int numberTicksAfterWalking = (int) Math.round(totalTimeAfterWalking / timePerTick);
      List<String> exceptions = createVariableNamesStringsToIgnore();
      SCS2RewindabilityVerifier checker = new SCS2RewindabilityVerifier(simulationTestHelper1, simulationTestHelper2, exceptions);
      double maxDifferenceAllowed = 1e-14;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<>();

      // checker.checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numberTicksBeforeWalking, maxDifferenceAllowed, variableDifferences);
      checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt,
                                                                                     numberTicksBeforeWalking,
                                                                                     maxDifferenceAllowed,
                                                                                     variableDifferences);
      checkForVariableDifferences(variableDifferences);
      YoBoolean walk1 = (YoBoolean) simulationTestHelper1.findVariable("walkCSG");
      YoBoolean walk2 = (YoBoolean) simulationTestHelper2.findVariable("walkCSG");
      walk1.set(true);
      walk2.set(true);

      // Must be greater than zero since the footstep provider stuff is not rewindable...
      numTicksToStartComparingAt = 1;

      // checker.checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numberTicksAfterWalking, maxDifferenceAllowed, variableDifferences);
      checker.checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(numTicksToStartComparingAt,
                                                                                     numberTicksAfterWalking,
                                                                                     maxDifferenceAllowed,
                                                                                     variableDifferences);
      checkForVariableDifferences(variableDifferences);

      simulationTestHelper1.finishTest();
      simulationTestHelper2.finishTest();
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
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

   private SCS2AvatarTestingSimulation setupSimulation()
   {
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = new SCS2AvatarTestingSimulationFactory(robotModel, environment);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(true, new HeadingAndVelocityEvaluationScriptParameters());
      simulationTestHelperFactory.setup(simulationTestingParameters);
      simulationTestHelperFactory.setRunMultiThreaded(false);
      SCS2AvatarTestingSimulation simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.setCameraFocusPosition(0.6, 0.4, 1.1);
      simulationTestHelper.setCameraPosition(-0.15, 10.0, 3.0);
      return simulationTestHelper;
   }

   private void initiateWalkingMotion(double standingTimeDuration, double walkingTimeDuration, SCS2AvatarTestingSimulation runner, YoBoolean walk)
   {
      walk.set(false);
      runner.simulateNow(standingTimeDuration);
      walk.set(true);
      runner.simulateNow(walkingTimeDuration);
   }

   public static List<String> createVariableNamesStringsToIgnore()
   {
      List<String> exceptions = new ArrayList<String>();
      exceptions.add("nano");
      exceptions.add("milli");
      exceptions.add("Timer");
      exceptions.add("startTime");
      exceptions.add("actualEstimatorDT");
      exceptions.add("nextExecutionTime");
      exceptions.add("totalDelay");
      exceptions.add("lastEstimatorClockStartTime");
      exceptions.add("lastControllerClockTime");
      exceptions.add("controllerStartTime");
      exceptions.add("actualControlDT");
      exceptions.add("timePassed");

      //    exceptions.add("gc_");
      //    exceptions.add("toolFrame");
      //    exceptions.add("ef_");
      //    exceptions.add("kp_");

      return exceptions;
   }
}
