package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.time.GlobalTimer;

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

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = false;
   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private RobotDataReceiver robotDataReceiver;
   private SDFRobot robot;
   private FullRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
      
      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCJunkyCommunicator");
      
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);

      fullRobotModel = drcBehaviorTestHelper.getFullRobotModel();

      robot = drcBehaviorTestHelper.getRobot();

      robotDataReceiver = drcBehaviorTestHelper.getRobotDataReceiver();
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testTwoStepsForwards() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());
      drcBehaviorTestHelper.dispatchBehavior(footstepListBehavior);

      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();
      SideDependentList<FramePose2d> desiredFootPoses = new SideDependentList<FramePose2d>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d desiredFootPose = getRobotFootPose2d(robot, robotSide);
         desiredFootPose.setX(desiredFootPose.getX() + 0.1);
         desiredFootPoses.set(robotSide, desiredFootPose);

         Footstep footStep = generateFootstep(desiredFootPoses.get(robotSide), fullRobotModel.getFoot(robotSide), fullRobotModel.getSoleFrame(robotSide),
               robotSide, 0.0, new Vector3d(0.0, 0.0, 1.0));
         desiredFootsteps.add(footStep);
      }

      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);
      assertTrue(footstepListBehavior.hasInputBeenSet());

      while (!footstepListBehavior.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
         assertTrue(success);
      }
      assertTrue(footstepListBehavior.isDone());

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2d finalFootPose = getRobotFootPose2d(robot, robotSide);
         assertPosesAreWithinThresholds(desiredFootPoses.get(robotSide), finalFootPose);
      }

      BambooTools.reportTestFinishedMessage();
   }

   private Footstep generateFootstep(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height, Vector3d planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3d position = new Point3d(footPose2d.getX(), footPose2d.getY(), height);
      Quat4d orientation = new Quat4d();
      RotationFunctions.getQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      Footstep footstep = new Footstep(foot, robotSide, soleFrame);
      footstep.setSolePose(new FramePose(ReferenceFrame.getWorldFrame(), position, orientation));

      return footstep;
   }

   private FramePose2d getRobotFootPose2d(SDFRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose2d ret = new FramePose2d();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Midfeet Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Midfeet Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }

}
