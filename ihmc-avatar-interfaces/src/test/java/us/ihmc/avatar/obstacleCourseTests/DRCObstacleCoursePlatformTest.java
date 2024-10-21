package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.DRCFlatGroundRewindabilityTest;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2RewindabilityVerifier;
import us.ihmc.avatar.testTools.scs2.SCS2RewindabilityVerifierWithStackTracing;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
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
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCoursePlatformTest implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected SCS2AvatarTestingSimulation simulationTestHelper;

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

   @Test
   public void testRunsTheSameWayTwiceJustStanding() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      simulationTestingParameters.setRunMultiThreaded(false);

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      SCS2AvatarTestingSimulation simulationTestHelper1 = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper1.start();
      simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      SCS2AvatarTestingSimulation simulationTestHelper2 = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper2.start();

      List<String> exceptions = DRCFlatGroundRewindabilityTest.createVariableNamesStringsToIgnore();

      SCS2RewindabilityVerifier checker = new SCS2RewindabilityVerifier(simulationTestHelper1, simulationTestHelper2, exceptions);

      //      setupCameraForWalkingOverSmallPlatform(scs1);
      //      setupCameraForWalkingOverSmallPlatform(scs2);

      SCS2RewindabilityVerifierWithStackTracing helper = null;
      boolean useVariableListenerTestHelper = false; // Make this true if you want to record a history of variable changes and find the first instance where they are different. Don't check in as true though since takes lots of time.

      if (useVariableListenerTestHelper)
      {
         helper = new SCS2RewindabilityVerifierWithStackTracing(simulationTestHelper1, simulationTestHelper2, exceptions);
         helper.setRecordDifferencesForSimOne(true);
         helper.setRecordDifferencesForSimTwo(true);
         helper.clearChangesForSimulations();
      }

      if (useVariableListenerTestHelper)
      {
         int numberOfTicks = 10;
         for (int i = 0; i < numberOfTicks; i++)
         {
            System.out.println("Tick : " + i);
            simulationTestHelper1.simulateOneBufferRecordPeriodNow();
            simulationTestHelper2.simulateOneBufferRecordPeriodNow();

            boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
            if (areTheVariableChangesDifferent)
               helper.printOutStackTracesOfFirstChangedVariable();
         }

      }

      double runTime = 5.0;

      boolean success = simulationTestHelper1.simulateNow(runTime);
      success = success && simulationTestHelper2.simulateNow(runTime);

      if (useVariableListenerTestHelper)
      {
         System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityHelper");

         boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
         if (areTheVariableChangesDifferent)
            helper.printOutStackTracesOfFirstChangedVariable();
      }

      System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityVerifier");
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

   @Test
   public void testWalkingOverSmallPlatformQuickly()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverSmallPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0); //2.0);

      ReferenceFrame rootFrame = simulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint3D pelvisPosition = new FramePoint3D(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.10);
      simulationTestHelper.publishToController(message);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingPastSmallPlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue("Robot had an exception, probably fell.", success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSidestepOverSmallPlatform()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM_TURNED;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverSmallPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0); //2.0);

      FramePoint3D desiredPosition = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame());
      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPosition.subZ(0.05);
      double trajectoryTime = 1.0;
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPosition.getZ());
      success &= simulationTestHelper.simulateNow(1.5);

      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      FootstepDataListMessage footstepDataList = createFootstepsForSideSteppingOverSmallPlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(11.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSidestepOverSmallWall()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_WALL;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverSmallPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0); //2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSideSteppingOverSmallWall();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(11.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.7944324216932475, -4.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOverSmallPlatform()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverSmallPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0); //2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOntoSmallPlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOffOfSmallPlatform();
         simulationTestHelper.publishToController(footstepDataList);

         success = success && simulationTestHelper.simulateNow(4.0);
      }

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOntoMediumPlatformToesTouching()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.MEDIUM_PLATFORM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverMediumPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOntoMediumPlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.0997851961824665, -5.797669618987603, 0.9903260891750866);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOffOfMediumPlatform()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOffOfMediumPlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfMediumPlatform();
      footstepDataList.setDefaultSwingDuration(1.1);
      footstepDataList.setDefaultTransferDuration(0.5);
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOffOfMediumPlatformSlowSteps()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOffOfMediumPlatform();

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(controllerFullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FrameQuaternion desiredChestFrameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      double leanAngle = 30.0;
      desiredChestFrameOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), -2.36, Math.toRadians(leanAngle), Math.toRadians(0.0));
      Quaternion desiredChestQuat = new Quaternion(desiredChestFrameOrientation);

      double trajectoryTime = 0.5;
      simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                 desiredChestQuat,
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 pelvisZUpFrame));

      success = success && simulationTestHelper.simulateNow(0.5);

      //      double heightOffset = 0.12;
      //      double heightTrajectoryTime = 0.5;
      //      ComHeightPacket comHeightPacket = new ComHeightPacket(heightOffset, heightTrajectoryTime);
      //      drcSimulationTestHelper.send(comHeightPacket);

      success = success && simulationTestHelper.simulateNow(1.0);

      double swingTime = 1.0;
      double transferTime = 0.6;
      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfMediumPlatformNarrowFootSpacing(swingTime, transferTime);
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(7.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOffOfLargePlatform()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      System.out.println(selectedLocation.getStartingLocationOffset().getAdditionalOffset());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOffOfLargePlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfLargePlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOntoLargePlatform()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.LARGE_PLATFORM;

      System.out.println(selectedLocation.getStartingLocationOffset().getAdditionalOffset());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOffOfLargePlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOntoLargePlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      // simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-5.45, -7.14, 1.17);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingOverSmallPlatform()
   {
      Point3D cameraFix = new Point3D(-3.0, -4.6, 0.8);
      Point3D cameraPosition = new Point3D(-11.5, -5.8, 2.5);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverMediumPlatform()
   {
      Point3D cameraFix = new Point3D(-3.9, -5.6, 0.55);
      Point3D cameraPosition = new Point3D(-7.5, -2.3, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOffOfMediumPlatform()
   {
      Point3D cameraFix = new Point3D(-3.9, -5.6, 0.55);
      Point3D cameraPosition = new Point3D(-7.6, -2.4, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOffOfLargePlatform()
   {
      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoSmallPlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-3.330, -5.093, 0.236), new Quaternion(-0.003, 0.015, 0.924, -0.382)),
            new Pose3D(new Point3D(-3.498, -4.928, 0.235), new Quaternion(-6.366E-4, -2.228E-4, 0.924, -0.382))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSideSteppingOverSmallPlatform()
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
      RotationTools.computeQuaternionFromYawAndZNormal(3.0 / 4.0 * Math.PI, verticalVector, orientation);
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.418, -5.012, 0.156), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.238, -4.832, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.518, -5.112, 0.156), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.388, -4.982, 0.156), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.688, -5.282, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.518, -5.112, 0.156), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-3.858, -5.452, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-3.688, -5.282, 0.0), orientation));
      return footstepDataList;
   }

   private FootstepDataListMessage createFootstepsForSideSteppingOverSmallWall()
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
      RotationTools.computeQuaternionFromYawAndZNormal(3.0 / 4.0 * Math.PI, verticalVector, orientation);
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.318, -3.912, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.198, -3.792, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                          new Point3D(-4.598, -4.192, 0.0),
                                                                          orientation,
                                                                          TrajectoryType.OBSTACLE_CLEARANCE,
                                                                          0.24));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.318, -3.912, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.728, -4.322, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                          new Point3D(-4.598, -4.192, 0.0),
                                                                          orientation,
                                                                          TrajectoryType.OBSTACLE_CLEARANCE,
                                                                          0.24));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(-4.858, -4.452, 0.0), orientation));
      footstepDataList.getFootstepDataList().add()
                      .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(-4.688, -4.282, 0.0), orientation));
      return footstepDataList;
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfSmallPlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-3.851, -5.250, 0.084), new Quaternion(-0.004, 0.004, 0.924, -0.383)),
            new Pose3D(new Point3D(-3.673, -5.447, 0.086), new Quaternion(-6.456E-5, -0.016, 0.923, -0.383))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingPastSmallPlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-3.330, -5.093, 0.236), new Quaternion(-0.003, 0.015, 0.923, -0.382)),
            new Pose3D(new Point3D(-3.850, -5.249, 0.084), new Quaternion(-0.003, 0.003, 0.923, -0.383)),
            new Pose3D(new Point3D(-3.672, -5.446, 0.085), new Quaternion(-6.456E-5, -0.015, 0.923, -0.383))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      FootstepDataListMessage desiredFootsteps = EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
      double zClearHeight = desiredFootsteps.getFootstepDataList().get(0).getLocation().getZ() + 0.07;
      double swingHeightForClear = zClearHeight - desiredFootsteps.getFootstepDataList().get(2).getLocation().getZ(); //should really be the last height (height before swing), not step 2, but they're approximate.
      desiredFootsteps.getFootstepDataList().get(1).setSwingHeight(swingHeightForClear);
      desiredFootsteps.getFootstepDataList().get(1).setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      return desiredFootsteps;
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoMediumPlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-4.144, -5.680, 0.284), new Quaternion(-0.012, 0.017, 0.923, -0.383)),
            new Pose3D(new Point3D(-3.997, -5.852, 0.292), new Quaternion(-0.022, -0.014, 0.923, -0.383))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfMediumPlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-4.304, -6.084, 0.087), new Quaternion(-0.004, -0.010, 0.924, -0.380)),
            new Pose3D(new Point3D(-4.439, -5.946, 0.085), new Quaternion(-8.975E-4, 0.002, 0.924, -0.380))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfMediumPlatformNarrowFootSpacing(double swingTime, double transferTime)
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-4.27, -5.67, 0.28), new Quaternion(-8.975E-4, 0.002, 0.924, -0.380)),
            new Pose3D(new Point3D(-4.34, -6.0, 0.087), new Quaternion(-0.004, -0.010, 0.924, -0.380)),
            new Pose3D(new Point3D(-4.5, -5.946, 0.085), new Quaternion(-8.975E-4, 0.002, 0.924, -0.380))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses, swingTime, transferTime);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfLargePlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-5.5 - 0.3 + 0.15, -7.171 - 0.3 - 0.15, 0.087), new Quaternion(-0.004, -0.010, 0.924, -0.380)),
            new Pose3D(new Point3D(-5.5 - 0.3 - 0.15, -7.171 - 0.3 + 0.15, 0.085), new Quaternion(-8.975E-4, 0.002, 0.924, -0.380)),};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoLargePlatform()
   {
      Pose3D[] footstepPoses = {new Pose3D(new Point3D(-5.5 + 0.15, -7.171 - 0.15, 0.4), new Quaternion(-0.004, -0.010, 0.924, -0.380)),
            new Pose3D(new Point3D(-5.5 - 0.15, -7.171 + 0.15, 0.4), new Quaternion(-8.975E-4, 0.002, 0.924, -0.380)),};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

}
