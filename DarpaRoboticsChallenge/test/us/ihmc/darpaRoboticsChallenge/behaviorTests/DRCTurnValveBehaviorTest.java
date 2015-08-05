package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.TurnValvePacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveTurnDirection;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspMethod;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.robotics.Axis;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCTurnValveBehaviorTest implements MultiRobotTestInterface
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private static final boolean DEBUG = false;

   @Before
   public void setUp()
   {
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   private static DRCValveEnvironment createTestEnvironment()
   {
      ArrayList<Point3d> valveLocations = new ArrayList<Point3d>();
      LinkedHashMap<Point3d, Double> valveYawAngles_degrees = new LinkedHashMap<Point3d, Double>();

      valveLocations.add(new Point3d(TurnValveBehavior.howFarToStandBackFromValve, TurnValveBehavior.howFarToStandToTheRightOfValve, 1.0));
      valveYawAngles_degrees.put(valveLocations.get(0), 0.0);

      valveLocations.add(new Point3d(TurnValveBehavior.howFarToStandBackFromValve, 4.0 * TurnValveBehavior.howFarToStandToTheRightOfValve, 1.0));
      valveYawAngles_degrees.put(valveLocations.get(1), 45.0);

      return new DRCValveEnvironment(valveLocations, valveYawAngles_degrees);
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testCloseValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);
      double initialValveClosePercentage = valveRobot.getClosePercentage();
      double turnValveThisMuchToCloseIt = valveRobot.getNumberOfPossibleTurns() * 2.0 * Math.PI * (1.0 - initialValveClosePercentage / 100.0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final TurnValveBehavior turnValveBehavior = createNewTurnValveBehavior();

      double graspApproachConeAngle = TurnValveBehavior.DEFAULT_GRASP_APPROACH_CONE_ANGLE;
      double valveRadius = ValveType.BIG_VALVE.getValveRadius();
      TurnValvePacket turnValvePacket = new TurnValvePacket(valveTransformToWorld, graspApproachConeAngle, valveRadius, 1.5 * turnValveThisMuchToCloseIt, TurnValveBehavior.DEFAULT_ROTATION_RATE_RAD_PER_SEC);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(turnValvePacket);
      assertTrue(turnValveBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(turnValveBehavior);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      PrintTools.debug(this, "Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & turnValveBehavior.isDone();

      assertTrue(success);
      assertTrue("Final valve close percentage, " + finalValveClosePercentage + ", is not greater than initial valve close percentage, "
            + initialValveClosePercentage + "!", finalValveClosePercentage > initialValveClosePercentage);
      assertTrue("Valve is not fully closed!  Final valve close percentage = " + finalValveClosePercentage, finalValveClosePercentage > 90.0);

      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold

      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testOpenValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);
      valveRobot.setClosePercentage(50.0);
      double initialValveClosePercentage = valveRobot.getClosePercentage();
      double turnValveThisMuchToOpenIt = -valveRobot.getNumberOfPossibleTurns() * 2.0 * Math.PI * (initialValveClosePercentage / 100.0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final TurnValveBehavior turnValveBehavior = createNewTurnValveBehavior();

      double graspApproachConeAngle = TurnValveBehavior.DEFAULT_GRASP_APPROACH_CONE_ANGLE;
      double valveRadius = ValveType.BIG_VALVE.getValveRadius();
      TurnValvePacket turnValvePacket = new TurnValvePacket(valveTransformToWorld, graspApproachConeAngle, valveRadius, 1.5 * turnValveThisMuchToOpenIt, turnValveBehavior.DEFAULT_ROTATION_RATE_RAD_PER_SEC);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(turnValvePacket);
      assertTrue(turnValveBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(turnValveBehavior);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      PrintTools.debug(this, "Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & turnValveBehavior.isDone();

      assertTrue(success);
      assertTrue("Final valve close percentage, " + finalValveClosePercentage + ", is greater than initial valve close percentage, "
            + initialValveClosePercentage + "!", finalValveClosePercentage < initialValveClosePercentage);
      assertTrue("Valve is not fully closed!  Final valve close percentage = " + finalValveClosePercentage, finalValveClosePercentage < 10.0);

      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testCloseValveByGrabbingCenter() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);
      double initialValveClosePercentage = valveRobot.getClosePercentage();
      double turnValveThisMuchToCloseIt = valveRobot.getNumberOfPossibleTurns() * 2.0 * Math.PI * (1.0 - initialValveClosePercentage / 100.0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final TurnValveBehavior turnValveBehavior = createNewTurnValveBehavior();

      double graspApproachConeAngle = Math.toRadians(0.0);
      double valveRadius = ValveType.BIG_VALVE.getValveRadius();
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(valveTransformToWorld, ValveGraspLocation.CENTER, ValveGraspMethod.RIM, graspApproachConeAngle, Axis.X, valveRadius, 1.5 * turnValveThisMuchToCloseIt, turnValveBehavior.DEFAULT_ROTATION_RATE_RAD_PER_SEC);
      assertTrue(turnValveBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(turnValveBehavior);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      PrintTools.debug(this, "Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & turnValveBehavior.isDone();

      assertTrue(success);
      assertTrue("Final valve close percentage, " + finalValveClosePercentage + ", is not greater than initial valve close percentage, "
            + initialValveClosePercentage + "!", finalValveClosePercentage > initialValveClosePercentage);
      assertTrue("Valve is not fully closed!  Final valve close percentage = " + finalValveClosePercentage, finalValveClosePercentage > 90.0);

      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkToAndCloseValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(1);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);
      double initialValveClosePercentage = valveRobot.getClosePercentage();
      double turnValveThisMuchToCloseIt = valveRobot.getNumberOfPossibleTurns() * 2.0 * Math.PI * (1.0 - initialValveClosePercentage / 100.0);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final TurnValveBehavior turnValveBehavior = createNewTurnValveBehavior();

      double graspApproachConeAngle = TurnValveBehavior.DEFAULT_GRASP_APPROACH_CONE_ANGLE;
      double valveRadius = ValveType.BIG_VALVE.getValveRadius();
      TurnValvePacket turnValvePacket = new TurnValvePacket(valveTransformToWorld, graspApproachConeAngle, valveRadius, 1.5 * turnValveThisMuchToCloseIt, turnValveBehavior.DEFAULT_ROTATION_RATE_RAD_PER_SEC);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(turnValvePacket);
      assertTrue(turnValveBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(turnValveBehavior);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      PrintTools.debug(this, "Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & turnValveBehavior.isDone();

      assertTrue(success);
      assertTrue("Final valve close percentage, " + finalValveClosePercentage + ", is not greater than initial valve close percentage, "
            + initialValveClosePercentage + "!", finalValveClosePercentage > initialValveClosePercentage);
      assertTrue("Valve is not fully closed!  Final valve close percentage = " + finalValveClosePercentage, finalValveClosePercentage > 90.0);

      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold
      //TODO: Test turning valve with a corrupted transform to world

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspValveBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);
     
      // Disable contact with valve
      ContactController valveContactController = (ContactController) valveRobot.getControllers().get(0).getController();
      valveContactController.setContactParameters(0.0, 0.0, 0.0, 0.0);
            
      RobotSide robotSideOfGraspingHand = RobotSide.RIGHT;
      boolean stopHandIfCollision = false;
      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final GraspValveBehavior graspValveBehavior = new GraspValveBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), getRobotModel(), drcBehaviorTestHelper.getYoTime());

      graspValveBehavior.initialize();
      graspValveBehavior.setGraspPose(robotSideOfGraspingHand, valveTransformToWorld, valveRobot.getValveRadius(), ValveGraspMethod.RIM, ValveTurnDirection.CLOCKWISE,
              TurnValveBehavior.DEFAULT_GRASP_APPROACH_CONE_ANGLE, Axis.X, stopHandIfCollision);
      FramePose desiredGraspPose = graspValveBehavior.getDesiredFinalGraspPose();
      PrintTools.debug(this, "Desired Final Grasp Pose: " + desiredGraspPose);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspValveBehavior);
      assertPosesAreWithinThresholds(desiredGraspPose, getCurrentHandPose(robotSideOfGraspingHand));

      success = success & graspValveBehavior.isDone();
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspValveUsingWholeBodyIKBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);
      
      // Disable contact with valve
      ContactController valveContactController = (ContactController) valveRobot.getControllers().get(0).getController();
      valveContactController.setContactParameters(0.0, 0.0, 0.0, 0.0);

      RobotSide robotSideOfGraspingHand = RobotSide.RIGHT;
      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      PrintTools.debug(this, "Valve Pose = " + valvePose);
      PrintTools.debug(this, "Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()));

      final GraspValveBehavior graspValveBehavior = new GraspValveBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), getRobotModel(), drcBehaviorTestHelper.getYoTime());

      graspValveBehavior.initialize();
      graspValveBehavior.setGraspPoseWholeBodyIK(robotSideOfGraspingHand, valveTransformToWorld, valveRobot.getValveRadius(),
            TurnValveBehavior.DEFAULT_GRASP_LOCATION, ValveTurnDirection.CLOCKWISE, TurnValveBehavior.DEFAULT_GRASP_APPROACH_CONE_ANGLE, Axis.X);
      FramePose desiredGraspPose = graspValveBehavior.getDesiredFinalGraspPose();
      PrintTools.debug(this, "Desired Final Grasp Pose: " + desiredGraspPose);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspValveBehavior);
      assertPosesAreWithinThresholds(desiredGraspPose, getCurrentHandPose(robotSideOfGraspingHand));

      success = success & graspValveBehavior.isDone();
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private TurnValveBehavior createNewTurnValveBehavior()
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      SDFFullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      CapturePointUpdatable capturePointUpdatable = drcBehaviorTestHelper.getCapturePointUpdatable();
      BooleanYoVariable yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      BooleanYoVariable yoTippingDetected = capturePointUpdatable.getTippingDetectedBoolean();
      WholeBodyControllerParameters wholeBodyControllerParameters = getRobotModel();
      WalkingControllerParameters walkingControllerParams = getRobotModel().getWalkingControllerParameters();

      final TurnValveBehavior turnValveBehavior = new TurnValveBehavior(communicationBridge, fullRobotModel, referenceFrames, yoTime, yoDoubleSupport,
            yoTippingDetected, wholeBodyControllerParameters);
      return turnValveBehavior;
   }

   private FramePose getRobotPose(HumanoidReferenceFrames referenceFrames)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose ret = new FramePose();

      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      ret.setToZero(midFeetFrame);
      ret.changeFrame(worldFrame);

      return ret;
   }

   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      drcBehaviorTestHelper.updateRobotModel();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose)
   {
      double positionThreshold = 0.05;
      double orientationThreshold = Math.toRadians(5.0);
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, orientationThreshold);
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         System.out.println("testSimpleHandPoseMove: positionDistance=" + positionDistance);
         System.out.println("testSimpleHandPoseMove: orientationDistance=" + orientationDistance);
      }

      assertEquals("Pose position error : " + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error : " + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
            orientationThreshold);
   }

}
