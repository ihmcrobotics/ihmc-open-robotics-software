package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Disabled
@Deprecated
public abstract class DRCFootstepListBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (behaviorTestHelper != null)
      {
         behaviorTestHelper.finishTest();
         behaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = false;
   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private SCS2BehaviorTestHelper behaviorTestHelper;
   private HumanoidRobotDataReceiver robotDataReceiver;

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), testEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);
      robotDataReceiver = behaviorTestHelper.getRobotDataReceiver();
   }

   @Test
   public void testTwoStepsForwards()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Sim");
      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      LogTools.info("Dispatching Behavior");
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(simulationTestHelper.getRobotName(),
                                                                           behaviorTestHelper.getROS2Node(),
                                                                           getRobotModel().getWalkingControllerParameters());
      behaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2D> desiredFootPoses = new SideDependentList<FramePose2D>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double xOffset = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
         Footstep desiredFootStep = generateFootstepOnFlatGround(robotSide, desiredFootPose);

         desiredFootPoses.set(robotSide, desiredFootPose);
         desiredFootsteps.add(desiredFootStep);
      }
      assertTrue(!areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps));

      LogTools.info("Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);
      success = simulationTestHelper.simulateNow(0.1);
      assertTrue(success);
      assertTrue(footstepListBehavior.hasInputBeenSet());
      assertTrue(footstepListBehavior.isWalking());

      LogTools.info("Begin Executing Behavior");
      while (!footstepListBehavior.isDone())
      {
         success = simulationTestHelper.simulateNow(1.0);
         assertTrue(success);
      }
      assertTrue(!footstepListBehavior.isWalking());
      assertTrue(footstepListBehavior.isDone());
      LogTools.info("Behavior should be done");

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D finalFootPose = getRobotFootPose2d(simulationTestHelper.getRobot(),
                                                        simulationTestHelper.getRobotModel().getJointMap().getFootName(robotSide));
         assertPosesAreWithinThresholds(desiredFootPoses.get(robotSide), finalFootPose);
      }

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSideStepping()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Sim");
      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      LogTools.info("Dispatching Behavior");
      behaviorTestHelper.updateRobotModel();
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(simulationTestHelper.getRobotName(),
                                                                           behaviorTestHelper.getROS2Node(),
                                                                           getRobotModel().getWalkingControllerParameters());
      behaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2D> desiredFootPoses = new SideDependentList<FramePose2D>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double yOffset = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, 0.0, yOffset);
         Footstep desiredFootStep = generateFootstepOnFlatGround(robotSide, desiredFootPose);

         desiredFootPoses.set(robotSide, desiredFootPose);
         desiredFootsteps.add(desiredFootStep);
      }
      assertTrue(!areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps));

      LogTools.info("Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);
      success = simulationTestHelper.simulateNow(0.1);
      assertTrue(success);
      assertTrue(footstepListBehavior.hasInputBeenSet());
      assertTrue(footstepListBehavior.isWalking());

      LogTools.info("Begin Executing Behavior");
      while (!footstepListBehavior.isDone())
      {
         success = simulationTestHelper.simulateNow(1.0);
         assertTrue(success);
      }
      assertTrue(!footstepListBehavior.isWalking());
      assertTrue(footstepListBehavior.isDone());
      LogTools.info("Behavior should be done");

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D finalFootPose = getRobotFootPose2d(simulationTestHelper.getRobot(),
                                                        simulationTestHelper.getRobotModel().getJointMap().getFootName(robotSide));
         assertPosesAreWithinThresholds(desiredFootPoses.get(robotSide), finalFootPose);
      }

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStepLongerThanMaxStepLength()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Sim");
      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      SideDependentList<FramePose2D> initialFootPoses = new SideDependentList<FramePose2D>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D initialFootPose = getRobotFootPose2d(simulationTestHelper.getRobot(),
                                                          simulationTestHelper.getRobotModel().getJointMap().getFootName(robotSide));
         initialFootPoses.put(robotSide, initialFootPose);
      }

      LogTools.info("Dispatching Behavior");
      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(simulationTestHelper.getRobotName(),
                                                                           behaviorTestHelper.getROS2Node(),
                                                                           getRobotModel().getWalkingControllerParameters());
      behaviorTestHelper.dispatchBehavior(footstepListBehavior);

      SideDependentList<FramePose2D> desiredFootPoses = new SideDependentList<FramePose2D>();
      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

      double xOffset = 1.5 * getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose2D desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
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

         FramePoint3D position = new FramePoint3D();
         FrameQuaternion orientation = new FrameQuaternion();
         footstep.getPose(position, orientation);

         RobotSide footstepSide = footstep.getRobotSide();
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(footstepSide, position, orientation);
         ret.getFootstepDataList().add().set(footstepData);
      }

      return ret;
   }

   private boolean areFootstepsTooFarApart(FootstepListBehavior footstepListBehavior, ArrayList<Footstep> desiredFootsteps)
   {
      ArrayList<Double> footStepLengths = footstepListBehavior.getFootstepLengths(createFootstepDataList(desiredFootsteps),
                                                                                  behaviorTestHelper.getSDFFullRobotModel(),
                                                                                  getRobotModel().getWalkingControllerParameters());

      if (DEBUG)
         for (double footStepLength : footStepLengths)
         {
            LogTools.info("foot step length : " + footStepLength);
         }

      boolean footStepsAreTooFarApart = footstepListBehavior.areFootstepsTooFarApart(createFootstepDataList(desiredFootsteps),
                                                                                     behaviorTestHelper.getSDFFullRobotModel(),
                                                                                     getRobotModel().getWalkingControllerParameters());

      return footStepsAreTooFarApart;
   }

   @Test
   public void testStop()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      LogTools.info("Initializing Sim");
      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      ArrayList<Double> xOffsets = new ArrayList<Double>();
      xOffsets.add(0.1);
      xOffsets.add(0.2);
      xOffsets.add(0.3);

      FootstepListBehavior footstepListBehavior = new FootstepListBehavior(simulationTestHelper.getRobotName(),
                                                                           behaviorTestHelper.getROS2Node(),
                                                                           getRobotModel().getWalkingControllerParameters());
      SideDependentList<FramePose2D> desiredFinalFootPoses = new SideDependentList<FramePose2D>();

      ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();
      for (double xOffset : xOffsets)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePose2D desiredFootPose = createFootPoseOffsetFromCurrent(robotSide, xOffset, 0.0);
            desiredFootsteps.add(generateFootstepOnFlatGround(robotSide, desiredFootPose));
            desiredFinalFootPoses.put(robotSide, desiredFootPose);
         }
      }

      areFootstepsTooFarApart(footstepListBehavior, desiredFootsteps);

      LogTools.info("Initializing Behavior");
      footstepListBehavior.initialize();
      footstepListBehavior.set(desiredFootsteps);

      LogTools.info("Begin Executing Behavior");
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stepNumberToStopOn = desiredFootsteps.size() - 1.0;
      double stopPercent = 100.0 * stepNumberToStopOn / desiredFootsteps.size();
      ReferenceFrame frameToKeepTrackOf = behaviorTestHelper.getReferenceFrames().getFootFrame(RobotSide.LEFT);
      StopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(robotDataReceiver,
                                                                                       footstepListBehavior,
                                                                                       pausePercent,
                                                                                       pauseDuration,
                                                                                       stopPercent,
                                                                                       desiredFinalFootPoses.get(RobotSide.LEFT),
                                                                                       frameToKeepTrackOf);
      behaviorTestHelper.executeBehaviorPauseAndResumeOrStop(footstepListBehavior, stopThreadUpdatable);
      LogTools.info("Behavior should be done");

      SideDependentList<FramePose2D> footPosesAtStop = new SideDependentList<FramePose2D>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = stopThreadUpdatable.getReferenceFramesAtTransition(BehaviorControlModeEnum.STOP).getFootFrame(robotSide);
         footPosesAtStop.put(robotSide, stopThreadUpdatable.getTestFramePose2dCopy(footFrame.getTransformToWorldFrame()));
      }

      SideDependentList<FramePose2D> footPosesFinal = new SideDependentList<FramePose2D>();
      for (RobotSide robotSide : RobotSide.values)
      {
         behaviorTestHelper.updateRobotModel();
         ReferenceFrame footFrame = behaviorTestHelper.getReferenceFrames().getFootFrame(robotSide);
         footPosesFinal.put(robotSide, stopThreadUpdatable.getTestFramePose2dCopy(footFrame.getTransformToWorldFrame()));
      }

      // Foot position and orientation may change after stop command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      for (RobotSide robotSide : RobotSide.values)
      {
         assertPosesAreWithinThresholds(footPosesAtStop.get(robotSide), footPosesFinal.get(robotSide), positionThreshold, orientationThreshold);
      }
      assertTrue(!footstepListBehavior.isDone());

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2D createFootPoseOffsetFromCurrent(RobotSide robotSide, double xOffset, double yOffset)
   {
      FramePose2D currentFootPose = getRobotFootPose2d(simulationTestHelper.getRobot(),
                                                       simulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getName());
      return createFootPoseOffsetFromExisting(xOffset, yOffset, currentFootPose);
   }

   private FramePose2D createFootPoseOffsetFromExisting(double xOffset, double yOffset, FramePose2D existingFootStep)
   {
      FramePose2D desiredFootPose = new FramePose2D(existingFootStep);
      desiredFootPose.setX(desiredFootPose.getX() + xOffset);
      desiredFootPose.setY(desiredFootPose.getY() + yOffset);

      return desiredFootPose;
   }

   private Footstep generateFootstepOnFlatGround(RobotSide robotSide, FramePose2D desiredFootPose2d)
   {
      Footstep ret = generateFootstep(desiredFootPose2d,
                                      behaviorTestHelper.getSDFFullRobotModel().getFoot(robotSide),
                                      behaviorTestHelper.getSDFFullRobotModel().getSoleFrame(robotSide),
                                      robotSide,
                                      0.0,
                                      new Vector3D(0.0, 0.0, 1.0));

      return ret;
   }

   private Footstep generateFootstep(FramePose2D footPose2d,
                                     RigidBodyBasics foot,
                                     ReferenceFrame soleFrame,
                                     RobotSide robotSide,
                                     double height,
                                     Vector3D planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3D position = new Point3D(footPose2d.getX(), footPose2d.getY(), height);
      Quaternion orientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      FramePose3D footstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(), position, orientation);
      Footstep footstep = new Footstep(robotSide, footstepPose);

      return footstep;
   }

   private FramePose2D getRobotFootPose2d(Robot robot, String footName)
   {

      FramePose2D ret = new FramePose2D();
      ret.setIncludingFrame(ReferenceFrame.getWorldFrame(), robot.getRigidBody(footName).getParentJoint().getFrameAfterJoint().getTransformToRoot(), false);

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         LogTools.info(" desired Midfeet Pose :\n" + desiredPose + "\n");
         LogTools.info(" actual Midfeet Pose :\n" + actualPose + "\n");

         LogTools.info(" positionDistance = " + positionDistance);
         LogTools.info(" orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold,
                   0.0,
                   orientationDistance,
                   orientationThreshold);
   }

}
