package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCFootstepListBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = false;
   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private HumanoidRobotDataReceiver robotDataReceiver;
   private HumanoidFloatingRootJointRobot robot;
   private FullHumanoidRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(),
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      robot = drcBehaviorTestHelper.getRobot();
      robotDataReceiver = drcBehaviorTestHelper.getRobotDataReceiver();
   }

   @ContinuousIntegrationTest(estimatedDuration = 31.2)
   @Test(timeout = 160000)
   public void testTwoStepsForwards() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PrintTools.debug(this, "Dispatching Behavior");
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),getRobotModel().getWalkingControllerParameters());
      drcBehaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2d> desiredFootPoses = new SideDependentList<FramePose2d>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double xOffset = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
         Footstep desiredFootStep = generateFootstepOnFlatGround(robotSide, desiredFootPose);

         desiredFootPoses.set(robotSide, desiredFootPose);
         desiredFootsteps.add(desiredFootStep);
      }
      assertTrue(!areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps));

      PrintTools.debug(this, "Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);
      assertTrue(footstepListBehavior.hasInputBeenSet());
      assertTrue(footstepListBehavior.isWalking());

      PrintTools.debug(this, "Begin Executing Behavior");
      while (!footstepListBehavior.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
         assertTrue(success);
      }
      assertTrue(!footstepListBehavior.isWalking());
      assertTrue(footstepListBehavior.isDone());
      PrintTools.debug(this, "Behavior should be done");

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d finalFootPose = getRobotFootPose2d(robot, robotSide);
         assertPosesAreWithinThresholds(desiredFootPoses.get(robotSide), finalFootPose);
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 31.9)
   @Test(timeout = 160000)
   public void testSideStepping() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PrintTools.debug(this, "Dispatching Behavior");
      drcBehaviorTestHelper.updateRobotModel();
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel().getWalkingControllerParameters());
      drcBehaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2d> desiredFootPoses = new SideDependentList<FramePose2d>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double yOffset = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, 0.0, yOffset);
         Footstep desiredFootStep = generateFootstepOnFlatGround(robotSide, desiredFootPose);

         desiredFootPoses.set(robotSide, desiredFootPose);
         desiredFootsteps.add(desiredFootStep);
      }
      assertTrue(!areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps));
      
      PrintTools.debug(this, "Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);
      assertTrue(footstepListBehavior.hasInputBeenSet());
      assertTrue(footstepListBehavior.isWalking());

      PrintTools.debug(this, "Begin Executing Behavior");
      while (!footstepListBehavior.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
         assertTrue(success);
      }
      assertTrue(!footstepListBehavior.isWalking());
      assertTrue(footstepListBehavior.isDone());
      PrintTools.debug(this, "Behavior should be done");

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d finalFootPose = getRobotFootPose2d(robot, robotSide);
         assertPosesAreWithinThresholds(desiredFootPoses.get(robotSide), finalFootPose);
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 26.1)
   @Test(timeout = 130000)
   public void testStepLongerThanMaxStepLength() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SideDependentList<FramePose2d> initialFootPoses = new SideDependentList<FramePose2d>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d initialFootPose = getRobotFootPose2d(robot, robotSide);
         initialFootPoses.put(robotSide, initialFootPose);
      }

      PrintTools.debug(this, "Dispatching Behavior");
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel().getWalkingControllerParameters());
      drcBehaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2d> desiredFootPoses = new SideDependentList<FramePose2d>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double xOffset = 1.5 * getRobotModel().getWalkingControllerParameters().getMaxStepLength();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
         Footstep desiredFootStep = generateFootstepOnFlatGround(robotSide, desiredFootPose);

         desiredFootPoses.set(robotSide, desiredFootPose);
         desiredFootsteps.add(desiredFootStep);
      }
      assertTrue(areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps));
   }

   private FootstepDataListMessage createFootstepDataList(ArrayList<Footstep> desiredFootsteps)
   {
      FootstepDataListMessage ret = new FootstepDataListMessage();

      for (int i = 0; i < desiredFootsteps.size(); i++)
      {
         Footstep footstep = desiredFootsteps.get(i);
         Point3D location = new Point3D(footstep.getX(), footstep.getY(), footstep.getZ());
         Quaternion orientation = new Quaternion();
         footstep.getOrientation(orientation);

         RobotSide footstepSide = footstep.getRobotSide();
         FootstepDataMessage footstepData = new FootstepDataMessage(footstepSide, location, orientation);
         ret.add(footstepData);
      }

      return ret;
   }

   private boolean areFootstepsTooFarApart(FootstepListBehavior footstepListBehavior, ArrayList<Footstep> desiredFootsteps)
   {
      ArrayList<Double> footStepLengths = footstepListBehavior.getFootstepLengths(createFootstepDataList(desiredFootsteps),
            drcBehaviorTestHelper.getSDFFullRobotModel(), getRobotModel().getWalkingControllerParameters());

      if(DEBUG)
      for (double footStepLength : footStepLengths)
      {
         PrintTools.debug(this, "foot step length : " + footStepLength);
      }
      
      boolean footStepsAreTooFarApart = footstepListBehavior.areFootstepsTooFarApart(createFootstepDataList(desiredFootsteps), fullRobotModel, getRobotModel().getWalkingControllerParameters());
      
      return footStepsAreTooFarApart;
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.1)
   @Test(timeout = 250000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ArrayList<Double> xOffsets = new ArrayList<Double>();
      xOffsets.add(0.1);
      xOffsets.add(0.2);
      xOffsets.add(0.3);

      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel().getWalkingControllerParameters());
      SideDependentList<FramePose2d> desiredFinalFootPoses = new SideDependentList<FramePose2d>();

      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();
      for (double xOffset : xOffsets)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePose2d desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
            desiredFootsteps.add(generateFootstepOnFlatGround(robotSide, desiredFootPose));
            desiredFinalFootPoses.put(robotSide, desiredFootPose);
         }
      }

      areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps);

      PrintTools.debug(this, "Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);

      PrintTools.debug(this, "Begin Executing Behavior");
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stepNumberToStopOn = desiredFootsteps.size() - 1.0;
      double stopPercent = 100.0 * stepNumberToStopOn / desiredFootsteps.size();
      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getFootFrame(RobotSide.LEFT);
      StopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(robotDataReceiver, footstepListBehavior, pausePercent, pauseDuration,
            stopPercent, desiredFinalFootPoses.get(RobotSide.LEFT), frameToKeepTrackOf);
      drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(footstepListBehavior, stopThreadUpdatable);
      PrintTools.debug(this, "Behavior should be done");

      SideDependentList<FramePose2d> footPosesAtStop = new SideDependentList<FramePose2d>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = stopThreadUpdatable.getReferenceFramesAtTransition(BehaviorControlModeEnum.STOP).getFootFrame(robotSide);
         footPosesAtStop.put(robotSide, stopThreadUpdatable.getTestFramePose2dCopy(footFrame.getTransformToWorldFrame()));
      }

      SideDependentList<FramePose2d> footPosesFinal = new SideDependentList<FramePose2d>();
      for (RobotSide robotSide : RobotSide.values)
      {
         drcBehaviorTestHelper.updateRobotModel();
         ReferenceFrame footFrame = drcBehaviorTestHelper.getReferenceFrames().getFootFrame(robotSide);
         footPosesFinal.put(robotSide, stopThreadUpdatable.getTestFramePose2dCopy(footFrame.getTransformToWorldFrame()));
      }

      // Foot position and orientation may change after stop command if the robot is currently in single support, 
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      for (RobotSide robotSide : RobotSide.values)
      {
         assertPosesAreWithinThresholds(footPosesAtStop.get(robotSide), footPosesFinal.get(robotSide), positionThreshold, orientationThreshold);
      }
      assertTrue(!footstepListBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2d createFootPoseOffsetFromCurrent(RobotSide robotSide, double xOffset, double yOffset)
   {
      FramePose2d currentFootPose = getRobotFootPose2d(robot, robotSide);
      return createFootPoseOffsetFromExisting(robotSide, xOffset, yOffset, currentFootPose);
   }

   private FramePose2d createFootPoseOffsetFromExisting(RobotSide robotSide, double xOffset, double yOffset, FramePose2d existingFootStep)
   {
      FramePose2d desiredFootPose = new FramePose2d(existingFootStep);
      desiredFootPose.setX(desiredFootPose.getX() + xOffset);
      desiredFootPose.setY(desiredFootPose.getY() + yOffset);

      return desiredFootPose;
   }

   private Footstep generateFootstepOnFlatGround(RobotSide robotSide, FramePose2d desiredFootPose2d)
   {
      Footstep ret = generateFootstep(desiredFootPose2d, fullRobotModel.getFoot(robotSide), fullRobotModel.getSoleFrame(robotSide), robotSide, 0.0,
            new Vector3D(0.0, 0.0, 1.0));

      return ret;
   }

   private Footstep generateFootstep(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height, Vector3D planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3D position = new Point3D(footPose2d.getX(), footPose2d.getY(), height);
      Quaternion orientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      Footstep footstep = new Footstep(foot, robotSide, soleFrame);
      footstep.setSolePose(new FramePose(ReferenceFrame.getWorldFrame(), position, orientation));

      return footstep;
   }

   private FramePose2d getRobotFootPose2d(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose2d ret = new FramePose2d();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose, double positionThreshold)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Midfeet Pose :\n" + desiredPose + "\n");
         PrintTools.debug(this, " actual Midfeet Pose :\n" + actualPose + "\n");

         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance, orientationThreshold);
   }

}
