package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentNavigableMap;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifierWithStackTracing;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Tag("humanoid-obstacle")
public abstract class DRCObstacleCoursePlatformTest implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testRunsTheSameWayTwiceJustStanding() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      simulationTestingParameters.setRunMultiThreaded(false);

      DRCSimulationTestHelper drcSimulationTestHelper1 = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper1.setStartingLocation(selectedLocation);
      drcSimulationTestHelper1.createSimulation("DRCWalkingOverSmallPlatformTest");
      DRCSimulationTestHelper drcSimulationTestHelper2 = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper2.setStartingLocation(selectedLocation);
      drcSimulationTestHelper2.createSimulation("DRCWalkingOverSmallPlatformTest");

      ArrayList<String> exceptions = DRCSimulationTestHelper.createVariableNamesStringsToIgnore();

      SimulationConstructionSet scs1 = drcSimulationTestHelper1.getSimulationConstructionSet();
      SimulationConstructionSet scs2 = drcSimulationTestHelper2.getSimulationConstructionSet();
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);

//      setupCameraForWalkingOverSmallPlatform(scs1);
//      setupCameraForWalkingOverSmallPlatform(scs2);

      SimulationRewindabilityVerifierWithStackTracing helper = null;
      boolean useVariableListenerTestHelper = false; // Make this true if you want to record a history of variable changes and find the first instance where they are different. Don't check in as true though since takes lots of time.

      if (useVariableListenerTestHelper)
      {
         helper = new SimulationRewindabilityVerifierWithStackTracing(scs1, scs2, exceptions);
         helper.setRecordDifferencesForSimOne(true);
         helper.setRecordDifferencesForSimTwo(true);
         helper.clearChangesForSimulations();
      }

      if (useVariableListenerTestHelper)
      {
         int numberOfTicks = 10;
         for (int i=0; i<numberOfTicks ; i++)
         {
            System.out.println("Tick : " + i);
            scs1.simulateOneRecordStepNow();
            scs2.simulateOneRecordStepNow();

            boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
            if (areTheVariableChangesDifferent) helper.printOutStackTracesOfFirstChangedVariable();
         }

      }

      double runTime = 5.0;

      boolean success = drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(runTime);
      success = success && drcSimulationTestHelper2.simulateAndBlockAndCatchExceptions(runTime);

      if (useVariableListenerTestHelper)
      {
         System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityHelper");

         boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
         if (areTheVariableChangesDifferent) helper.printOutStackTracesOfFirstChangedVariable();
      }

      System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityVerifier");
      ArrayList<VariableDifference> variableDifferences = checker.verifySimulationsAreSameToStart();

      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         if (simulationTestingParameters.getKeepSCSUp())
            ThreadTools.sleepForever();
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }


      drcSimulationTestHelper1.destroySimulation();
      drcSimulationTestHelper2.destroySimulation();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOverSmallPlatformQuickly() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      ReferenceFrame rootFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint3D pelvisPosition = new FramePoint3D(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.10);
      drcSimulationTestHelper.publishToController(message);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingPastSmallPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue("Robot had an exception, probably fell.", success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSidestepOverSmallPlatform() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM_TURNED;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSideSteppingOverSmallPlatform();
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(11.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSidestepOverSmallWall() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_WALL;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSideSteppingOverSmallWall();
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(11.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.7944324216932475, -4.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOverSmallPlatform() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOntoSmallPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOffOfSmallPlatform(scriptedFootstepGenerator);
         drcSimulationTestHelper.publishToController(footstepDataList);

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOntoMediumPlatformToesTouching() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.MEDIUM_PLATFORM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverMediumPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOntoMediumPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.0997851961824665, -5.797669618987603, 0.9903260891750866);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   @Test
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
      drcSimulationTestHelper = new DRCSimulationTestHelper( simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOffOfMediumPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfMediumPlatform(scriptedFootstepGenerator);
      footstepDataList.setDefaultSwingDuration(1.1);
      footstepDataList.setDefaultTransferDuration(0.5);
      footstepDataList.getFootstepDataList().get(1).setTransferSplitFraction(0.01);
      footstepDataList.getFootstepDataList().get(0).setSwingDurationShiftFraction(0.999);
      footstepDataList.getFootstepDataList().get(0).setSwingSplitFraction(0.75);
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   @Test
	public void testWalkingOffOfMediumPlatformSlowSteps() throws SimulationExceededMaximumTimeException
	{
	   simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
	   BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

	   DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
	   drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
	   drcSimulationTestHelper.setStartingLocation(selectedLocation);
	   drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

	   SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
	   ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

	   setupCameraForWalkingOffOfMediumPlatform(simulationConstructionSet);

	   FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(controllerFullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

	   ThreadTools.sleep(1000);
	   boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

	   FrameQuaternion desiredChestFrameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
	   double leanAngle = 30.0;
	   desiredChestFrameOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), -2.36, Math.toRadians(leanAngle), Math.toRadians(0.0));
      Quaternion desiredChestQuat = new Quaternion(desiredChestFrameOrientation);

      double trajectoryTime = 0.5;
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestQuat, ReferenceFrame.getWorldFrame(), pelvisZUpFrame));

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      double heightOffset = 0.12;
      double heightTrajectoryTime = 0.5;
//      ComHeightPacket comHeightPacket = new ComHeightPacket(heightOffset, heightTrajectoryTime);
//      drcSimulationTestHelper.send(comHeightPacket);

	   success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

	   double swingTime = 1.0;
      double transferTime = 0.6;
      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfMediumPlatformNarrowFootSpacing(scriptedFootstepGenerator, swingTime, transferTime);
	   drcSimulationTestHelper.publishToController(footstepDataList);

	   success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

	   drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
	   drcSimulationTestHelper.checkNothingChanged();

	   assertTrue(success);

	   Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
	   Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
	   BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
	   drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

	   BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
	}

	private void setupCameraForWalkingOverSmallPlatform(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-3.0, -4.6, 0.8);
      Point3D cameraPosition = new Point3D(-11.5, -5.8, 2.5);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverMediumPlatform(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-3.9, -5.6, 0.55);
      Point3D cameraPosition = new Point3D(-7.5, -2.3, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOffOfMediumPlatform(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-3.9, -5.6, 0.55);
      Point3D cameraPosition = new Point3D(-7.6, -2.4, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private FootstepDataListMessage createFootstepsForSteppingOntoSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.3303508964136372, -5.093152916934431, 0.2361869051765919}, {-0.003380023644676521, 0.01519186055257256, 0.9239435001894032, -0.3822122332825927}},
            {{-3.4980005080184333, -4.927710662235891, 0.23514263035532196}, {-6.366244432153206E-4, -2.2280928201561157E-4, 0.9240709626189128, -0.3822203567445069}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSideSteppingOverSmallPlatform()
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
      RotationTools.computeQuaternionFromYawAndZNormal(3.0/4.0*Math.PI, verticalVector, orientation);
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.418, -5.012, 0.156), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.238, -4.832, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.518, -5.112, 0.156), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.388, -4.982, 0.156), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.688, -5.282, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.518, -5.112, 0.156), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.858, -5.452, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.688, -5.282, 0.0), new Quaternion(orientation)));
      return footstepDataList;
   }

   private FootstepDataListMessage createFootstepsForSideSteppingOverSmallWall()
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
      RotationTools.computeQuaternionFromYawAndZNormal(3.0/4.0*Math.PI, verticalVector, orientation);
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.318, -3.912, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.198, -3.792, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.598, -4.192, 0.0), new Quaternion(orientation), TrajectoryType.OBSTACLE_CLEARANCE, 0.24));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.318, -3.912, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.728, -4.322, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.598, -4.192, 0.0), new Quaternion(orientation), TrajectoryType.OBSTACLE_CLEARANCE, 0.24));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.858, -4.452, 0.0), new Quaternion(orientation)));
      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.688, -4.282, 0.0), new Quaternion(orientation)));
      return footstepDataList;
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.850667406347062, -5.249955436839419, 0.08402883817600326}, {-0.0036296745847858064, 0.003867481752280881, 0.9236352342329301, -0.38323598752046323}},
            {{-3.6725725349280296, -5.446807690769805, 0.08552806597763604}, {-6.456929194763128E-5, -0.01561897825296648, 0.9234986484659182, -0.3832835629540643}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSteppingPastSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.3303508964136372, -5.093152916934431, 0.2361869051765919}, {-0.003380023644676521, 0.01519186055257256, 0.9239435001894032, -0.3822122332825927}},
            {{-3.850667406347062, -5.249955436839419, 0.08402883817600326}, {-0.0036296745847858064, 0.003867481752280881, 0.9236352342329301, -0.38323598752046323}},
            {{-3.6725725349280296, -5.446807690769805, 0.08552806597763604}, {-6.456929194763128E-5, -0.01561897825296648, 0.9234986484659182, -0.3832835629540643}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      FootstepDataListMessage desiredFootsteps = scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
      double zClearHeight = desiredFootsteps.getFootstepDataList().get(0).getLocation().getZ() + 0.07;
      double swingHeightForClear = zClearHeight - desiredFootsteps.getFootstepDataList().get(2).getLocation().getZ(); //should really be the last height (height before swing), not step 2, but they're approximate.
      desiredFootsteps.getFootstepDataList().get(1).setSwingHeight(swingHeightForClear);
      desiredFootsteps.getFootstepDataList().get(1).setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      return desiredFootsteps;
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoMediumPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-4.144889177599215, -5.68009276450442, 0.2841471307289875}, {-0.012979910123161926, 0.017759854548746876, 0.9232071519598507, -0.3836726001029824}},
            {{-3.997325285359919, -5.8527640256176685, 0.2926905844610473}, {-0.022159348866436335, -0.014031420240348416, 0.9230263369316307, -0.3838417171627259}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfMediumPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-4.304392715667327, -6.084498586699763, 0.08716704456087025}, {-0.0042976203878775715, -0.010722204803598987, 0.9248070170408506, -0.38026115501738456}},
            {{-4.4394706079327255, -5.9465856725464565, 0.08586305720146342}, {-8.975861226689934E-4, 0.002016837110644428, 0.9248918980282926, -0.380223754740342}},
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfMediumPlatformNarrowFootSpacing(ScriptedFootstepGenerator scriptedFootstepGenerator, double swingTime, double transferTime)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-4.27, -5.67, 0.28}, {-8.975861226689934E-4, 0.002016837110644428, 0.9248918980282926, -0.380223754740342}},
            {{-4.34, -6.0, 0.08716704456087025}, {-0.0042976203878775715, -0.010722204803598987, 0.9248070170408506, -0.38026115501738456}},
            {{-4.5, -5.9465856725464565, 0.08586305720146342}, {-8.975861226689934E-4, 0.002016837110644428, 0.9248918980282926, -0.380223754740342}},
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations, swingTime, transferTime);
   }



}
