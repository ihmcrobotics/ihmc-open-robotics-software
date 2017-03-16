package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HumanoidPointyRocksTest implements MultiRobotTestInterface
{
   private final static double defaultSwingTime = 0.6;
   private final static double defaultTransferTime = 2.5;
   private final static double defaultChickenPercentage = 0.5;

   private final YoVariableRegistry registry = new YoVariableRegistry("PointyRocksTest");
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SideDependentList<YoFrameConvexPolygon2d> supportPolygons = null;
   private SideDependentList<ArrayList<Point2D>> footContactsInAnkleFrame = null;

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private PushRobotController pushController;

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final double[] rightHandStraightSideJointAngles = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242, 0.6123343067479899, 0.12524505635696856};
   private static final double[] leftHandStraightSideJointAngles = new double[] {0.61130147334225, 0.22680071472282162, 1.6270339908033258, 1.2703560974484844, 0.10340544060719102, -0.6738299572358809, 0.13264785356924128};
   private static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }

   private void setUpMomentum() throws SimulationExceededMaximumTimeException
   {
      // enable the use of body momentum in the controller
      BooleanYoVariable allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInSingleSupport.set(true);
      BooleanYoVariable allowUsingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");
      allowUsingHighMomentumWeight.set(true);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.robotSide = robotSide;
         double[] armConfig = straightArmConfigs.get(robotSide);
         armTrajectoryMessage.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[armConfig.length];
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.position = armConfig[i];
            trajectoryPoint.time = 1.0;
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.trajectoryPoints = new TrajectoryPoint1DMessage[] {trajectoryPoint};
            armTrajectoryMessage.jointTrajectoryMessages[i] = jointTrajectory;
         }
         drcSimulationTestHelper.send(armTrajectoryMessage);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);
   }

   /**
    * The real robot often falls to the outside when taking a step with only line contact on the support foot. To avoid falling to the outside
    * angular momentum can be used. This test simulates the scenario by pushing the robot to the support side during swing. Without the use of
    * upper body angular momentum the robot will fall over.
    */
   public void testSidePushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.5;
      double transferTime = 0.0;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      BooleanYoVariable l_swingSpeedup = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("leftFootIsSwingSpeedUpEnabled");
      BooleanYoVariable r_swingSpeedup = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("rightFootIsSwingSpeedUpEnabled");
      l_swingSpeedup.set(false);
      r_swingSpeedup.set(false);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double ankleDamping = 1.0;
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      for (RobotSide robotSide : RobotSide.values)
      {
         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName).setDamping(ankleDamping);
         simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName).setDamping(ankleDamping);
      }

      setupCameraForWalkingUpToRamp();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      setUpMomentum();

      // take one step forward onto a line contact
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      SideDependentList<String> footJointNames = getFootJointNames(fullRobotModel);
      double stepLength = 0.6;
      FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(RobotSide.LEFT), stepLength/2.0, 0.0, 0.0);
      ArrayList<Point2D> contacts = generateContactPointsForRotatedLineOfContact(0.0);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, RobotSide.LEFT, contacts, stepLocation, footJointNames, true, swingTime, transferTime);

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      stepLocation.setIncludingFrame(fullRobotModel.getSoleFrame(RobotSide.RIGHT), stepLength, 0.0, 0.0);
      contacts = generateContactPointsForAllOfFoot();
      FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel, RobotSide.RIGHT, contacts, stepLocation, true);
      message.add(footstepData);
      drcSimulationTestHelper.send(message);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);

      Vector3D rootVelocity = new Vector3D();
      FloatingJoint rootJoint = robot.getRootJoint();
      rootJoint.getVelocity(rootVelocity);
      double push = 0.04;
      rootVelocity.setY(rootVelocity.getY() + push);
      rootJoint.setVelocity(rootVelocity);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testBalanceOnLine() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      pushController = new PushRobotController(drcSimulationTestHelper.getRobot(), drcSimulationTestHelper.getRobot().getRootJoint().getName(), new Vector3D(0.0, 0.0, 0.15));
      ThreadTools.sleep(1000);
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);
      DoubleYoVariable momentumGain = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("momentumGain");
      momentumGain.set(0.5);

      BooleanYoVariable l_swingSpeedup = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("leftFootIsSwingSpeedUpEnabled");
      BooleanYoVariable r_swingSpeedup = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("rightFootIsSwingSpeedUpEnabled");
      l_swingSpeedup.set(false);
      r_swingSpeedup.set(false);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double ankleDamping = 1.0;
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      for (RobotSide robotSide : RobotSide.values)
      {
         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName).setDamping(ankleDamping);
         simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName).setDamping(ankleDamping);
      }

      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 5.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      drcSimulationTestHelper.getSimulationConstructionSet().hideAllYoGraphics();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      setUpMomentum();

      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      // take one step forward onto a line contact
      FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(RobotSide.LEFT), 0.0, 0.0, 0.0);
      double lineWidth = 0.01;
      ArrayList<Point2D> contacts = generateContactPointsForRotatedLineOfContact(lineWidth, 0.0);
      double offset = -0.04;
      for (Point2D contact : contacts)
         contact.setY(contact.getY() + offset);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, RobotSide.LEFT, contacts , stepLocation, jointNames, true, defaultSwingTime, defaultTransferTime);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      FramePoint desiredPosition = new FramePoint(fullRobotModel.getSoleFrame(RobotSide.RIGHT), 0.0, 0.0, 0.15);
      desiredPosition.changeFrame(worldFrame);
      Quaternion desiredOrientation = new Quaternion();
      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(RobotSide.RIGHT, 1.0, desiredPosition.getPoint(), desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);

      pushController.applyForce(new Vector3D(0.0, 1.0, 0.0), 200.0, 0.025);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);

      Point3D center = new Point3D(-0.0198, 0.1317, 0.7865);
      Vector3D plusMinusVector = new Vector3D(0.025, 0.025, 0.025);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      assertTrue(success);
   }

   /**
    * This test takes one step at a time (throw-catch). For each step, the predicted contact points are randomly changed to be a
    * thin knife edge. However, the true contact points remain the full foot. So this tests that the ICP planner and such work well
    * when you know the contact conditions. It does not, however, test detection of the contact conditions, or the ability to hold
    * the foot's orientation without collapsing into a void.
    */
   public void testWalkingWithLinePredictedSupportPolygonButFullActualPolygon() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 0.5;
      double transferTime = 0.0;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      // should make the test more robust
      BooleanYoVariable allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) drcSimulationTestHelper.getSimulationConstructionSet().getVariable("allowUpperBodyMomentumInSingleSupport");
      BooleanYoVariable allowUpperBodyMomentumInDoubleSupport = (BooleanYoVariable) drcSimulationTestHelper.getSimulationConstructionSet().getVariable("allowUpperBodyMomentumInDoubleSupport");
      BooleanYoVariable allowUsingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getSimulationConstructionSet().getVariable("allowUsingHighMomentumWeight");
      allowUsingHighMomentumWeight.set(true);
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsWithRandomPredictedContactPointLines(scriptedFootstepGenerator, swingTime, transferTime);

      for (FootstepDataListMessage footstepDataList : footstepDataLists)
      {
         drcSimulationTestHelper.send(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.4277546756235229, 0.2801160933192322, 0.7880809572883962);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test steps in place with the actual and the predicted foot polygons changing to be the given foot shrinkage percentage.
    * This test therefore tests the ability to hold orientation of the foot without over-rotating. However, it does not test
    * the ability to detect the actual ground contact conditions since they are told through the predicted.
    */
   public void testTakingStepsWithActualAndPredictedFootPolygonsChanging() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);

      Point3D rootPositionAtStart = new Point3D();
      drcSimulationTestHelper.getRobot().getRootJoint().getPosition(rootPositionAtStart);

      int numberOfSteps = 5;
      double footShrinkagePercentWidth = 0.25;
      double footShrinkagePercentLength = 0.75;
      boolean setPredictedContactPoints = true;

      for (int i = 0; i < numberOfSteps; i++)
      {
         RobotSide robotSide = RobotSide.LEFT;
         FramePoint stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));
         ArrayList<Point2D> contactPointsInAnkleFrame = generateContactPointsForSlightlyPulledInAnkleFrame(walkingControllerParameters, footShrinkagePercentWidth, footShrinkagePercentLength);
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

         robotSide = RobotSide.RIGHT;
         stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));
         contactPointsInAnkleFrame = generateContactPointsForSlightlyPulledInAnkleFrame(walkingControllerParameters, footShrinkagePercentWidth, footShrinkagePercentLength);
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);
      }

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Vector3D plusMinusVector = new Vector3D(0.01, 0.01, 0.01);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(rootPositionAtStart, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test stands one one leg and then gets pushed sideways. The true ground contact is slightly narrower than what the controller thinks.
    * Without the push, all is fine. But with the push, the robot will over-rotate the foot if hold position isn't working well.
    */
   public void testHoldPositionByStandingOnOneLegAndGettingPushedSideways() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setUsePefectSensors(false);

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);
      setupSupportViz();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double widthPercentage = 0.85;
      double lengthPercentage = 1.0;
      ArrayList<Point2D> newContactPoints = generateContactPointsForSlightlyPulledInAnkleFrame(getRobotModel().getWalkingControllerParameters(), widthPercentage, lengthPercentage);
      String jointName = "l_leg_akx";
      changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointName, RobotSide.LEFT);

      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      RobotSide robotSide = RobotSide.RIGHT;
      FootTrajectoryCommand liftFootCommand = new FootTrajectoryCommand();
      liftFootCommand.setRobotSide(robotSide);

      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      FramePoint footPosition = new FramePoint(ankleFrame);
      footPosition.changeFrame(worldFrame);

      footPosition.setZ(footPosition.getZ() + 0.1);

      liftFootCommand.addTrajectoryPoint(1.0, footPosition.getPointCopy(), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      queuedControllerCommands.add(liftFootCommand);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      FloatingJoint rootJoint = robot.getRootJoint();
      rootJoint.setVelocity(0.0, 0.1, 0.0);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * In this test, the robot is standing, but then the floor is dropped out from underneath it. So the robot has to detect the rotation
    * and hold position. Then it takes some steps in place with the part of foot changing each step.
    */
   public void testStandingAndStepsInPlaceWithHalfFootContactsChanges() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper);

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide robotSide = RobotSide.LEFT;
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);

      ArrayList<Point2D> newContactPoints = generateContactPointsForLeftOfFoot(getRobotModel().getWalkingControllerParameters(), 0.5);
      changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointNames.get(RobotSide.LEFT), RobotSide.LEFT);
      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      boolean setPredictedContactPoints = false;
      FramePoint stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);
      double percentOfFootToKeep = 0.5;

      newContactPoints = generateContactPointsForBackOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      newContactPoints = generateContactPointsForRightOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      robotSide = RobotSide.RIGHT;

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

      newContactPoints = generateContactPointsForBackOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      newContactPoints = generateContactPointsForRightOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints, defaultSwingTime, defaultTransferTime);

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * In this test, the robot walks forward. On each step a part of the foot is cut away. The controller knows about the foothold. No exploration is done
    */
   public void testWalkingForwardWithPartialFootholdsAndStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double transferTime = 0.0;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      setupSupportViz();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      boolean setPredictedContactPoints = true;

      Vector3D stepVector = new Vector3D();

      stepVector.set(0.2, 0.0, 0.0);

      int numberOfSteps = 20;
      ArrayList<Point2D> newContactPoints;

      Random random = new Random(1984L);
      for (int i=0; i<numberOfSteps; i++)
      {
         // Type of contact change options:
         //  0    uniform shrinking
         //  1    line of contact
         //  2    half foot
         int typeOfContactChange = random.nextInt(3);

         if (typeOfContactChange == 0)
         {
            double shrinkageLengthPercent = RandomNumbers.nextDouble(random, 0.5, 0.6);
            double shrinkageWidthPercent = RandomNumbers.nextDouble(random, 0.5, 0.6);
            newContactPoints = generateContactPointsForUniformFootShrinkage(getRobotModel().getWalkingControllerParameters(), shrinkageLengthPercent, shrinkageWidthPercent);
         }
         else if (typeOfContactChange == 1)
         {
            newContactPoints = generateContactPointsForRandomRotatedLineOfContact(random);
         }
         else if (typeOfContactChange == 2)
         {
            double percentOfFootToKeep = RandomNumbers.nextDouble(random, 0.0, 0.5);
            newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
         }
         else
         {
            throw new RuntimeException("Should not go here");
         }

         double stepLength = 0.3;
         double stepWidth = 0.3;

         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);

         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, setPredictedContactPoints, defaultSwingTime, transferTime);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

         robotSide = robotSide.getOppositeSide();
      }

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   /**
    * In this test, the robot walks forward. On each step a half of the foot is cut out.
    */
   public void testWalkingForwardWithHalfFootContactChangesStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 0.5;
      double transferTime = 0.0;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, 0.2);

      setupSupportViz();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      boolean setPredictedContactPoints = false;

      Vector3D stepVector = new Vector3D();

      stepVector.set(0.2, 0.0, 0.0);

      int numberOfSteps = 5;
      ArrayList<Point2D> newContactPoints;

      Random random = new Random(1984L);
      for (int i=0; i<numberOfSteps; i++)
      {
         // Type of contact change options:
         //  0    uniform shrinking
         //  1    line of contact
         //  2    half foot
         int typeOfContactChange = random.nextInt(3);

         if (typeOfContactChange == 0)
         {
            double shrinkageLengthPercent = RandomNumbers.nextDouble(random, 0.5, 0.6);
            double shrinkageWidthPercent = RandomNumbers.nextDouble(random, 0.5, 0.6);
            newContactPoints = generateContactPointsForUniformFootShrinkage(getRobotModel().getWalkingControllerParameters(), shrinkageLengthPercent, shrinkageWidthPercent);
         }
         else if (typeOfContactChange == 1)
         {
            newContactPoints = generateContactPointsForRandomRotatedLineOfContact(random);
         }
         else if (typeOfContactChange == 2)
         {
            double percentOfFootToKeep = RandomNumbers.nextDouble(random, 0.3, 0.6);
            newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
         }
         else
         {
            throw new RuntimeException("Should not go here");
         }

         double stepLength = 0.3;
         double stepWidth = 0.3;

         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);

         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, setPredictedContactPoints, swingTime, transferTime);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

         robotSide = robotSide.getOppositeSide();
      }

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * In this test, the robot walks forward. On each step a half of the foot is cut out. The steps are continuous with no stopping in between steps.
    */
   public void testWalkingForwardWithHalfFootContactChangesContinuousSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 0.5;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, 0.15);

      setupCameraForWalkingUpToRamp();
//      setupSupportViz();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);
      boolean setPredictedContactPoints = false;

      ArrayList<Point2D> newContactPoints;
      double stepLength = 0.0;
      double stepWidth = 0.3;

      for (RobotSide robotSide : RobotSide.values)
      {
         newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), 0.5);
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);

         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, setPredictedContactPoints, swingTime, defaultTransferTime);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      }

      int numberOfSteps = 5;
      RobotSide robotSide = RobotSide.LEFT;
      FootstepDataListMessage message = new FootstepDataListMessage();
      stepLength = 0.5;

      for (int i=0; i<numberOfSteps; i++)
      {
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel, robotSide, null, stepLocation, false);
         message.add(footstepData);
         robotSide = robotSide.getOppositeSide();
      }

      drcSimulationTestHelper.send(message);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * 2.0);

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test will drop the floor out from underneath the sim randomly while standing. Tests if detection and hold position are working well.
    */
   public void testStandingWithGCPointsChangingOnTheFly() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(1984L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper);

      // Since the foot support points change while standing, the parts of the support polygon that need to be cut off might have had the CoP in them.
      BooleanYoVariable useCoPOccupancyGrid = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("ExplorationFoothold_UseCopOccupancyGrid");
      useCoPOccupancyGrid.set(false);
      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      setupCameraForWalkingUpToRamp();
//      setupSupportViz();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);
      HighLevelHumanoidControllerToolbox controllerToolbox = drcSimulationTestHelper.getAvatarSimulation().getMomentumBasedControllerFactory()
                                                                                          .getHighLevelHumanoidControllerToolbox();

      int numberOfChanges = 4;

      for (int i=0; i<numberOfChanges; i++)
      {
         ArrayList<Point2D> newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), 0.4);
         changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointNames.get(robotSide), robotSide);
         success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
         if (!success) break;

         // check if the found support polygon is close to the actual one
         FrameConvexPolygon2d foundSupport = controllerToolbox.getBipedSupportPolygons().getFootPolygonInSoleFrame(robotSide);
         FrameConvexPolygon2d actualSupport = new FrameConvexPolygon2d(foundSupport.getReferenceFrame(), newContactPoints);
         double epsilon = 5.0; // cm^2
         boolean close = Math.abs(foundSupport.getArea() - actualSupport.getArea()) * 10000 < epsilon;
         if (!close) {
            System.out.println("Area expected: " + actualSupport.getArea()*10000 + " [cm^2]");
            System.out.println("Area found:    " + foundSupport.getArea()*10000  + " [cm^2]");
         }
         assertTrue("Support polygon found does not match the actual one.", close);

         // step in place to reset robot
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), 0.0, 0.0, 0.0);
         newContactPoints = generateContactPointsForAllOfFoot();
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, true, defaultSwingTime, defaultTransferTime);
         if (!success) break;
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.06095496955280358, -0.001119333179390724, 0.7875020745919501);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private SideDependentList<String> getFootJointNames(FullHumanoidRobotModel fullRobotModel)
   {
      SideDependentList<String> jointNames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         jointNames.put(robotSide, fullRobotModel.getFoot(robotSide).getParentJoint().getName());
      }
      return jointNames;
   }

   private void enablePartialFootholdDetectionAndResponse(DRCSimulationTestHelper drcSimulationTestHelper)
   {
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
   }

   private void enablePartialFootholdDetectionAndResponse(DRCSimulationTestHelper drcSimulationTestHelper, double chickenPercentage)
   {
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = fullRobotModel.getFoot(robotSide).getName();
         BooleanYoVariable doPartialFootholdDetection = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable(footName + "DoPartialFootholdDetection");
         doPartialFootholdDetection.set(true);

         // Set joint velocity limits on the ankle joints so that they don't flip out when doing partial footsteps. On real robots with joint velocity limits, this should
         // happen naturally.
         double qd_max = 12.0;
         double b_vel_limit = 500.0;

         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         if (simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName) instanceof PinJoint)
         {
            PinJoint ankleJoint = (PinJoint) simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName);
            ankleJoint.setVelocityLimits(qd_max , b_vel_limit);
         }
         else
         {
            throw new RuntimeException("Can not set velocity limits on ankle joint " + firstAnkleName + " - it is not a PinJoint.");
         }

         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         if (simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName) instanceof PinJoint)
         {
            PinJoint ankleJoint = (PinJoint) simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName);
            ankleJoint.setVelocityLimits(qd_max , b_vel_limit);
         }
         else
         {
            throw new RuntimeException("Can not set velocity limits on ankle joint " + secondAnkleName + " - it is not a PinJoint.");
         }
      }

      BooleanYoVariable doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(true);

      DoubleYoVariable percentageChickenSupport = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("PercentageChickenSupport");
      percentageChickenSupport.set(chickenPercentage);

      DoubleYoVariable timeBeforeExploring = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("ExplorationState_TimeBeforeExploring");
      timeBeforeExploring.set(0.0);
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<FootstepDataListMessage> createFootstepsWithRandomPredictedContactPointLines(ScriptedFootstepGenerator scriptedFootstepGenerator, double swingTime, double transferTime)
   {
      ArrayList<FootstepDataListMessage> messages = new ArrayList<>();
      Random random = new Random(1776L);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ReferenceFrame ankleFrame, soleFrame;
      RobotSide robotSide;

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(0.50, 0.10, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.LEFT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage(swingTime, transferTime);
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(1.0, -0.10, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.RIGHT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage(swingTime, transferTime);
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(1.5, 0.10, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.LEFT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage(swingTime, transferTime);
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(1.5, -0.1, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.RIGHT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage(swingTime, transferTime);
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(1.5, 0.4, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.LEFT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage(swingTime, transferTime);
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3D(1.5, 0.2, 0.0));
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      robotSide = RobotSide.RIGHT;
      footstepData.setRobotSide(robotSide);
      ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      footstepData.setPredictedContactPoints(transformFromAnkleFrameToSoleFrame(generateContactPointsForRandomRotatedLineOfContact(random), ankleFrame, soleFrame));
      message.add(footstepData);
      messages.add(message);

      return messages;
   }

   private ArrayList<Point2D> generateContactPointsForSlightlyPulledInAnkleFrame(WalkingControllerParameters walkingControllerParameters, double widthPercentage, double lengthPercentage)
   {
      double footForwardOffset = lengthPercentage * walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = lengthPercentage * walkingControllerParameters.getFootBackwardOffset();
      double footWidth = widthPercentage * walkingControllerParameters.getFootWidth();
      double toeWidth = widthPercentage * walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(HumanoidFloatingRootJointRobot robot, FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         ArrayList<Point2D> contactPointsInAnkleFrame, FramePoint placeToStep, SideDependentList<String> jointNames, boolean setPredictedContactPoints, double swingTime, double transferTime)
         throws SimulationExceededMaximumTimeException
   {
      return takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, contactPointsInAnkleFrame, placeToStep,
            jointNames, setPredictedContactPoints, swingTime, transferTime);
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(HumanoidFloatingRootJointRobot robot, FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         ArrayList<Point2D> contactPointsInAnkleFrame, ArrayList<Point2D> predictedContactPointsInAnkleFrame, FramePoint placeToStep,
         SideDependentList<String> jointNames, boolean setPredictedContactPoints, double swingTime, double transferTime) throws SimulationExceededMaximumTimeException
   {
      String jointName = jointNames.get(robotSide);

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel, robotSide, predictedContactPointsInAnkleFrame, placeToStep, setPredictedContactPoints);
      message.add(footstepData);

      drcSimulationTestHelper.send(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.2);
      changeAppendageGroundContactPointsToNewOffsets(robot, contactPointsInAnkleFrame, jointName, robotSide);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      return success;
   }

   private FootstepDataMessage createFootstepDataMessage(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, ArrayList<Point2D> contactPointsInAnkleFrame, FramePoint placeToStep,
         boolean setPredictedContactPoints)
   {
      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);

      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint placeToStepInWorld = new FramePoint(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld.getPointCopy());
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

      if (setPredictedContactPoints && (contactPointsInAnkleFrame != null))
      {
         ArrayList<Point2D> contactPointsInSoleFrame = transformFromAnkleFrameToSoleFrame(contactPointsInAnkleFrame, ankleFrame, soleFrame);
         footstepData.setPredictedContactPoints(contactPointsInSoleFrame);
      }
      return footstepData;
   }

   private void changeAppendageGroundContactPointsToNewOffsets(HumanoidFloatingRootJointRobot robot, ArrayList<Point2D> newContactPoints, String jointName, RobotSide robotSide)
   {
      double time = robot.getTime();
      System.out.println("Changing contact points at time " + time);

      int pointIndex = 0;
      ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

      for (GroundContactPoint point : allGroundContactPoints)
      {
         Joint parentJoint = point.getParentJoint();

         if (parentJoint.getName().equals(jointName))
         {
            Point2D newContactPoint = newContactPoints.get(pointIndex);

            point.setIsInContact(false);
            Vector3D offset = new Vector3D();
            point.getOffset(offset);
            //            System.out.println("originalOffset = " + offset);

            offset.setX(newContactPoint.getX());
            offset.setY(newContactPoint.getY());

            //            System.out.println("newOffset = " + offset);
            point.setOffsetJoint(offset);

            pointIndex++;
         }
      }

      if (footContactsInAnkleFrame != null)
      {
         footContactsInAnkleFrame.set(robotSide, newContactPoints);
      }
   }

   private ArrayList<Point2D> transformFromAnkleFrameToSoleFrame(ArrayList<Point2D> originalPoints, ReferenceFrame ankleFrame, ReferenceFrame soleFrame)
   {
      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      for (Point2D originalPoint : originalPoints)
      {
         FramePoint framePoint = new FramePoint(ankleFrame, originalPoint.getX(), originalPoint.getY(), 0.0);
         framePoint.changeFrame(soleFrame);

         ret.add(new Point2D(framePoint.getX(), framePoint.getY()));
      }

      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForAllOfFoot()
   {
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForUniformFootShrinkage(WalkingControllerParameters walkingControllerParameters, double lengthPercent, double widthPercent)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      ret.add(new Point2D(lengthPercent * footForwardOffset, widthPercent * toeWidth / 2.0));
      ret.add(new Point2D(lengthPercent * footForwardOffset, -widthPercent * toeWidth / 2.0));
      ret.add(new Point2D(-lengthPercent * footBackwardOffset, -widthPercent * footWidth / 2.0));
      ret.add(new Point2D(-lengthPercent * footBackwardOffset, widthPercent * footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForHalfOfFoot(Random random, WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      int footHalf = random.nextInt(4);

      if (footHalf == 0)
         return generateContactPointsForLeftOfFoot(walkingControllerParameters, percentToKeep);
      if (footHalf == 1)
         return generateContactPointsForRightOfFoot(walkingControllerParameters, percentToKeep);
      if (footHalf == 2)
         return generateContactPointsForFrontOfFoot(walkingControllerParameters, percentToKeep);

      return generateContactPointsForBackOfFoot(walkingControllerParameters, percentToKeep);
   }

   private ArrayList<Point2D> generateContactPointsForLeftOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double width = percentToKeep * (toeWidth + footWidth) / 2.0;
      ret.add(new Point2D(footForwardOffset, width - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, 0.0));
      ret.add(new Point2D(-footBackwardOffset, 0.0));
      ret.add(new Point2D(-footBackwardOffset, width - footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForRightOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double width = percentToKeep * (toeWidth + footWidth) / 2.0;
      ret.add(new Point2D(footForwardOffset, width - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, width - footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForFrontOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();
      double footLength = walkingControllerParameters.getFootLength();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double length = percentToKeep * (footLength);
      double widthAtBack = percentToKeep * footWidth + (1-percentToKeep) * toeWidth;
      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset - length, - widthAtBack / 2.0));
      ret.add(new Point2D(footForwardOffset - length, widthAtBack / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForBackOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();
      double footLength = walkingControllerParameters.getFootLength();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double length = percentToKeep * (footLength);
      double widthAtFront = percentToKeep * toeWidth + (1-percentToKeep) * footWidth;
      ret.add(new Point2D(-footBackwardOffset + length, widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset + length, - widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset, - footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForRandomRotatedLineOfContact(Random random)
   {
      return generateContactPointsForRotatedLineOfContact(RandomNumbers.nextDouble(random, Math.PI));
   }

   private ArrayList<Point2D> generateContactPointsForRotatedLineOfContact(double angle)
   {
      return generateContactPointsForRotatedLineOfContact(0.01, angle);
   }

   private ArrayList<Point2D> generateContactPointsForRotatedLineOfContact(double width, double angle)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(angle);

      Line2d leftLine = new Line2d(new Point2D(0.0, width / 2.0), new Vector2D(1.0, 0.0));
      Line2d rightLine = new Line2d(new Point2D(0.0, -width / 2.0), new Vector2D(1.0, 0.0));

      leftLine.applyTransform(transform);
      rightLine.applyTransform(transform);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2d solePolygon = new ConvexPolygon2d(soleVertices);
      solePolygon.update();

      Point2D[] leftIntersections = solePolygon.intersectionWith(leftLine);
      Point2D[] rightIntersections = solePolygon.intersectionWith(rightLine);

      ArrayList<Point2D> ret = new ArrayList<Point2D>();
      ret.add(leftIntersections[0]);
      ret.add(leftIntersections[1]);
      ret.add(rightIntersections[0]);
      ret.add(rightIntersections[1]);
      return ret;
   }

   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);

   private void setupSupportViz()
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      supportPolygons = new SideDependentList<YoFrameConvexPolygon2d>();
      supportPolygons.set(RobotSide.LEFT, new YoFrameConvexPolygon2d("FootPolygonLeft", "", worldFrame, 4, registry));
      supportPolygons.set(RobotSide.RIGHT, new YoFrameConvexPolygon2d("FootPolygonRight", "", worldFrame, 4, registry));

      footContactsInAnkleFrame = new SideDependentList<ArrayList<Point2D>>();
      footContactsInAnkleFrame.set(RobotSide.LEFT, null);
      footContactsInAnkleFrame.set(RobotSide.RIGHT, null);

      yoGraphicsListRegistry.registerArtifact("SupportLeft", new YoArtifactPolygon("SupportLeft", supportPolygons.get(RobotSide.LEFT), Color.BLACK, false));
      yoGraphicsListRegistry.registerArtifact("SupportRight", new YoArtifactPolygon("SupportRight", supportPolygons.get(RobotSide.RIGHT), Color.BLACK, false));

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      drcSimulationTestHelper.addRobotControllerOnControllerThread(new VizUpdater());
   }

   private class VizUpdater implements RobotController
   {
      FrameConvexPolygon2d footSupport = new FrameConvexPolygon2d(worldFrame);
      FramePoint2d point = new FramePoint2d(worldFrame);
      FramePoint point3d = new FramePoint();

      @Override
      public void doControl()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ArrayList<Point2D> contactPoints = footContactsInAnkleFrame.get(robotSide);
            if (contactPoints == null) continue;

            footSupport.clear(worldFrame);
            ReferenceFrame ankleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getEndEffectorFrame(robotSide, LimbName.LEG);
            ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrames().get(robotSide);

            for (int i = 0; i < contactPoints.size(); i++)
            {
               point3d.setXYIncludingFrame(ankleFrame, contactPoints.get(i));
               point3d.changeFrame(soleFrame);
               point3d.setZ(0.0);
               point3d.changeFrame(worldFrame);
               point3d.getFramePoint2d(point);
               footSupport.addVertex(point.getPoint());
            }

            footSupport.update();
            supportPolygons.get(robotSide).setFrameConvexPolygon2d(footSupport);
         }
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return null;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }
   };
}
