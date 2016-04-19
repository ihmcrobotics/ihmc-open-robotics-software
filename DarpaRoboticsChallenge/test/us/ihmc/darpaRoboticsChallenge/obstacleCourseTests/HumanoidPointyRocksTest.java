package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HumanoidPointyRocksTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
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

   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * This test takes one step at a time (throw-catch). For each step, the predicted contact points are randomly changed to be a 
    * thin knife edge. However, the true contact points remain the full foot. So this tests that the ICP planner and such work well
    * when you know the contact conditions. It does not, however, test detection of the contact conditions, or the ability to hold
    * the foot's orientation without collapsing into a void.
    */
   public void testWalkingWithLinePredictedSupportPolygonButFullActualPolygon() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsWithRandomPredictedContactPointLines(scriptedFootstepGenerator);

      for (FootstepDataListMessage footstepDataList : footstepDataLists)
      {
         drcSimulationTestHelper.send(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(1.4277546756235229, 0.2801160933192322, 0.7880809572883962);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * This test steps in place with the actual and the predicted foot polygons changing to be the given foot shrinkage percentage.
    * This test therefore tests the ability to hold orientation of the foot without over-rotating. However, it does not test
    * the ability to detect the actual ground contact conditions since they are told through the predicted.
    */
   public void testTakingStepsWithActualAndPredictedFootPolygonsChanging() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      SideDependentList<String> jointNames = new SideDependentList<>("l_leg_akx", "r_leg_akx");

      int numberOfSteps = 5;
      double footShrinkagePercentWidth = 0.25;
      double footShrinkagePercentLength = 0.75;
      for (int i = 0; i < numberOfSteps; i++)
      {
         boolean setPredictedContactPoints = true;
         RobotSide robotSide = RobotSide.LEFT;
         FramePoint stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

         ArrayList<Point2d> contactPointsInAnkleFrame = generateContactPointsForSlightlyPulledInAnkleFrame(walkingControllerParameters, footShrinkagePercentWidth, footShrinkagePercentLength);
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, stepInPlaceLocation, jointNames, setPredictedContactPoints);

         robotSide = RobotSide.RIGHT;
         stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

         contactPointsInAnkleFrame = generateContactPointsForSlightlyPulledInAnkleFrame(walkingControllerParameters, footShrinkagePercentWidth, footShrinkagePercentLength);
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, stepInPlaceLocation, jointNames, setPredictedContactPoints);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * This test stands one one leg and then gets pushed sideways. The true ground contact is slightly narrower than what the controller thinks.
    * Without the push, all is fine. But with the push, the robot will over-rotate the foot if hold position isn't working well.
    */
   public void testHoldPositionByStandingOnOneLegAndGettingPushedSideways() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setUsePefectSensors(false);

      BambooTools.reportTestStartedMessage();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double widthPercentage = 0.85;
      double lengthPercentage = 1.0;
      ArrayList<Point2d> newContactPoints = generateContactPointsForSlightlyPulledInAnkleFrame(getRobotModel().getWalkingControllerParameters(), widthPercentage, lengthPercentage);
      String jointName = "l_leg_akx";
      changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointName);

      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      RobotSide robotSide = RobotSide.RIGHT;
      FootTrajectoryCommand liftFootCommand = new FootTrajectoryCommand();
      liftFootCommand.setRobotSide(robotSide);

      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      FramePoint footPosition = new FramePoint(ankleFrame);
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      footPosition.setZ(footPosition.getZ() + 0.1);

      liftFootCommand.addTrajectoryPoint(1.0, footPosition.getPointCopy(), new Quat4d(0.0, 0.0, 0.0, 1.0), new Vector3d(), new Vector3d());
      queuedControllerCommands.add(liftFootCommand);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      FloatingJoint rootJoint = robot.getRootJoint();
      rootJoint.setVelocity(0.0, 0.1, 0.0);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * In this test, the robot is standing, but then the floor is dropped out from underneath it. So the robot has to detect the rotation
    * and hold position. Then it takes some steps in place with the part of foot changing each step.
    */
   public void testStandingAndStepsInPlaceWithHalfFootContactsChanges() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      
      RobotSide robotSide = RobotSide.LEFT;
      SideDependentList<String> jointNames = new SideDependentList<>("l_leg_akx", "r_leg_akx");

      ArrayList<Point2d> newContactPoints = generateContactPointsForLeftOfFoot(getRobotModel().getWalkingControllerParameters());
      changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointNames.get(RobotSide.LEFT));
      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      
      boolean setPredictedContactPoints = false;      
      FramePoint stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);
      double percentOfFootToKeep = 0.25;

      newContactPoints = generateContactPointsForBackOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);

      newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);
      
      newContactPoints = generateContactPointsForRightOfFoot(getRobotModel().getWalkingControllerParameters());
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);

      robotSide = RobotSide.RIGHT;
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      stepInPlaceLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));

      newContactPoints = generateContactPointsForBackOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);

      newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);
      
      newContactPoints = generateContactPointsForRightOfFoot(getRobotModel().getWalkingControllerParameters());
      success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepInPlaceLocation, jointNames, setPredictedContactPoints);

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * In this test, the robot walks forward. On each step a half of the foot is cut out.
    */
   public void testWalkingForwardWithHalfFootContactChangesStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      
      RobotSide robotSide = RobotSide.LEFT;
      SideDependentList<String> jointNames = new SideDependentList<>("l_leg_akx", "r_leg_akx");
      boolean setPredictedContactPoints = false;

      Vector3d stepVector = new Vector3d();

      stepVector.set(0.2, 0.0, 0.0);

      int numberOfSteps = 10;
      ArrayList<Point2d> newContactPoints;
      double percentOfFootToKeep = 0.0; //0.5;

      Random random = new Random(1984L);
      for (int i=0; i<numberOfSteps; i++)
      {         
         newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);          
//         newContactPoints = generateContactPointsForUniformFootShrinkage(getRobotModel().getWalkingControllerParameters(), 0.5, 0.5);
         
         double stepLength = 0.3;
         double stepWidth = 0.3;

         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);

         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, setPredictedContactPoints);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

         robotSide = robotSide.getOppositeSide();
      }
      
      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * In this test, the robot walks forward. On each step a half of the foot is cut out. The steps are continuous with no stopping in between steps.
    */
   public void testWalkingForwardWithHalfFootContactChangesContinuousSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SideDependentList<String> jointNames = new SideDependentList<>("l_leg_akx", "r_leg_akx");
      boolean setPredictedContactPoints = false;

      ArrayList<Point2d> newContactPoints;
      double stepLength = 0.0;
      double stepWidth = 0.3;

      for (RobotSide robotSide : RobotSide.values)
      {         
         double percentOfFootToKeep = 0.0;
         newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters(), percentOfFootToKeep);          
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()), stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);

         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, setPredictedContactPoints);
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
      BambooTools.reportTestFinishedMessage();
   }


   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   /**
    * This test will drop the floor out from underneath the sim randomly while standing. Tests if detection and hold position are working well.
    */
   public void testStandingWithGCPointsChangingOnTheFly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      Random random = new Random(1984L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      String jointName = "l_leg_akx";

      int numberOfChanges = 4;
      
      for (int i=0; i<numberOfChanges; i++)
      {
         ArrayList<Point2d> newContactPoints = generatePredictedContactPoints(random);
         changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointName);
         success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
         if (!success) break;
      }
           
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(-0.06095496955280358, -0.001119333179390724, 0.7875020745919501);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<FootstepDataListMessage> createFootstepsWithRandomPredictedContactPointLines(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      ArrayList<FootstepDataListMessage> messages = new ArrayList<>();
      Random random = new Random(1776L);

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(0.50, 0.10, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.LEFT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage();
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(1.0, -0.10, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.RIGHT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage();
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(1.5, 0.10, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.LEFT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage();
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(1.5, -0.1, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.RIGHT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage();
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(1.5, 0.4, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.LEFT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      message = new FootstepDataListMessage();
      footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setLocation(new Point3d(1.5, 0.2, 0.0));
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(RobotSide.RIGHT);
      footstepData.setPredictedContactPoints(generatePredictedContactPoints(random));
      message.add(footstepData);
      messages.add(message);

      return messages;
   }

   private ArrayList<Point2d> generateContactPointsForSlightlyPulledInAnkleFrame(WalkingControllerParameters walkingControllerParameters, double widthPercentage, double lengthPercentage)
   {
      double footForwardOffset = lengthPercentage * walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = lengthPercentage * walkingControllerParameters.getFootBackwardOffset();
      double footWidth = widthPercentage * walkingControllerParameters.getFootWidth();
      double toeWidth = widthPercentage * walkingControllerParameters.getToeWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      ret.add(new Point2d(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2d(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(SDFHumanoidRobot robot, 
         SDFFullHumanoidRobotModel fullRobotModel, RobotSide robotSide, 
         ArrayList<Point2d> contactPointsInAnkleFrame, FramePoint placeToStep,
         SideDependentList<String> jointNames, boolean setPredictedContactPoints)
         throws SimulationExceededMaximumTimeException
   {
      String jointName = jointNames.get(robotSide);

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel, robotSide, contactPointsInAnkleFrame, placeToStep, setPredictedContactPoints);
      message.add(footstepData);

      drcSimulationTestHelper.send(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.6);
      changeAppendageGroundContactPointsToNewOffsets(robot, contactPointsInAnkleFrame, jointName);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      return success;
   }

   private FootstepDataMessage createFootstepDataMessage(SDFFullHumanoidRobotModel fullRobotModel, RobotSide robotSide, ArrayList<Point2d> contactPointsInAnkleFrame, FramePoint placeToStep,
         boolean setPredictedContactPoints)
   {
      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);

      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint placeToStepInWorld = new FramePoint(placeToStep);
      placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      
      footstepData.setLocation(placeToStepInWorld.getPointCopy());
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      if (setPredictedContactPoints && (contactPointsInAnkleFrame != null))
      {
         ArrayList<Point2d> contactPointsInSoleFrame = transformFromAnkleFrameToSoleFrame(contactPointsInAnkleFrame, ankleFrame, soleFrame);
         footstepData.setPredictedContactPoints(contactPointsInSoleFrame);
      }
      return footstepData;
   }

   private void changeAppendageGroundContactPointsToNewOffsets(SDFHumanoidRobot robot, ArrayList<Point2d> newContactPoints, String jointName)
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
            Point2d newContactPoint = newContactPoints.get(pointIndex);

            point.setIsInContact(false);
            Vector3d offset = new Vector3d();
            point.getOffset(offset);
            //            System.out.println("originalOffset = " + offset);

            offset.setX(newContactPoint.getX());
            offset.setY(newContactPoint.getY());

            //            System.out.println("newOffset = " + offset);
            point.setOffsetJoint(offset);

            pointIndex++;
         }
      }
   }

   private ArrayList<Point2d> transformFromAnkleFrameToSoleFrame(ArrayList<Point2d> originalPoints, ReferenceFrame ankleFrame, ReferenceFrame soleFrame)
   {
      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      for (Point2d originalPoint : originalPoints)
      {
         FramePoint framePoint = new FramePoint(ankleFrame, originalPoint.getX(), originalPoint.getY(), 0.0);
         framePoint.changeFrame(soleFrame);

         ret.add(new Point2d(framePoint.getX(), framePoint.getY()));
      }

      return ret;
   }

   private ArrayList<Point2d> generateContactPointsForAllOfFoot(WalkingControllerParameters walkingControllerParameters)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      
      ret.add(new Point2d(footForwardOffset, footWidth / 2.0));
      ret.add(new Point2d(footForwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }
   
   private ArrayList<Point2d> generateContactPointsForUniformFootShrinkage(WalkingControllerParameters walkingControllerParameters, double lengthPercent, double widthPercent)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      
      ret.add(new Point2d(lengthPercent * footForwardOffset, widthPercent * footWidth / 2.0));
      ret.add(new Point2d(lengthPercent * footForwardOffset, -widthPercent * footWidth / 2.0));
      ret.add(new Point2d(-lengthPercent * footBackwardOffset, -widthPercent * footWidth / 2.0));
      ret.add(new Point2d(-lengthPercent * footBackwardOffset, widthPercent * footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2d> generateContactPointsForHalfOfFoot(Random random, WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      int footHalf = random.nextInt(4);
      
      if (footHalf == 0)
         return generateContactPointsForLeftOfFoot(walkingControllerParameters);
      if (footHalf == 1)
         return generateContactPointsForRightOfFoot(walkingControllerParameters);
      if (footHalf == 2)
         return generateContactPointsForFrontOfFoot(walkingControllerParameters, percentToKeep);
     
      return generateContactPointsForBackOfFoot(walkingControllerParameters, percentToKeep);
   }
   
   private ArrayList<Point2d> generateContactPointsForSideOfFoot(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters)
   {
      if (robotSide == RobotSide.LEFT)
      {
         return generateContactPointsForLeftOfFoot(getRobotModel().getWalkingControllerParameters());          
      }
      else
      {
         return generateContactPointsForRightOfFoot(getRobotModel().getWalkingControllerParameters());            
      }
   }
   
   private ArrayList<Point2d> generateContactPointsForLeftOfFoot(WalkingControllerParameters walkingControllerParameters)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      
      ret.add(new Point2d(footForwardOffset, footWidth / 2.0));
      ret.add(new Point2d(footForwardOffset, 0.0));
      ret.add(new Point2d(-footBackwardOffset, 0.0));
      ret.add(new Point2d(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }
   
   private ArrayList<Point2d> generateContactPointsForRightOfFoot(WalkingControllerParameters walkingControllerParameters)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      
      ret.add(new Point2d(footForwardOffset, 0.0));
      ret.add(new Point2d(footForwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, 0.0));
      return ret;
   }

   private ArrayList<Point2d> generateContactPointsForFrontOfFoot(WalkingControllerParameters walkingControllerParameters, double percentOfBackOfFootToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      
      double newFootBackwardOffset = footForwardOffset / 2.0 + (percentOfBackOfFootToKeep * (-footBackwardOffset - footForwardOffset/2.0));
      
      ret.add(new Point2d(newFootBackwardOffset, footWidth / 2.0));
      ret.add(new Point2d(newFootBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(footForwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(footForwardOffset, footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2d> generateContactPointsForBackOfFoot(WalkingControllerParameters walkingControllerParameters, double percentOfFrontOfFootToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();

      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      ret.add(new Point2d(footForwardOffset * percentOfFrontOfFootToKeep, footWidth / 2.0));
      ret.add(new Point2d(footForwardOffset * percentOfFrontOfFootToKeep, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2d(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2d> generatePredictedContactPoints(Random random)
   {
      double angle = RandomTools.generateRandomDouble(random, Math.PI);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.rotZ(angle);

      Line2d leftLine = new Line2d(new Point2d(0.0, 0.005), new Vector2d(1.0, 0.0));
      Line2d rightLine = new Line2d(new Point2d(0.0, -0.005), new Vector2d(1.0, 0.0));

      leftLine.applyTransform(transform);
      rightLine.applyTransform(transform);

      ArrayList<Point2d> soleVertices = new ArrayList<Point2d>();
      soleVertices.add(new Point2d(0.1, 0.05));
      soleVertices.add(new Point2d(0.1, -0.05));
      soleVertices.add(new Point2d(-0.1, -0.05));
      soleVertices.add(new Point2d(-0.1, 0.05));
      ConvexPolygon2d solePolygon = new ConvexPolygon2d(soleVertices);
      solePolygon.update();

      Point2d[] leftIntersections = solePolygon.intersectionWith(leftLine);
      Point2d[] rightIntersections = solePolygon.intersectionWith(rightLine);

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      ret.add(leftIntersections[0]);
      ret.add(leftIntersections[1]);
      ret.add(rightIntersections[0]);
      ret.add(rightIntersections[1]);
      return ret;
   }

   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);
}
