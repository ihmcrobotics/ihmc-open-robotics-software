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
import org.junit.Ignore;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
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

   @Ignore
   @DeployableTestMethod(estimatedDuration = 71.5)
   @Test(timeout = 360000)
   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String name = "DRCQueuedControllerCommandTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      //       BooleanYoVariable walk = (BooleanYoVariable) robot.getVariable("walk");
      //       walk.set(true);

      FootstepDataListCommand footstepList = new FootstepDataListCommand();
      FootstepDataControllerCommand footstepCommand = new FootstepDataControllerCommand();

      Point3d position = new Point3d(0.0, 0.2, 0.0);
      Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(0.3, -0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(0.8, 0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(0.8, -0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);

      footstepList = new FootstepDataListCommand();
      footstepCommand = new FootstepDataControllerCommand();

      position = new Point3d(1.0, 0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(1.3, -0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(1.8, 0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3d(1.8, -0.2, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(1.8, 0.0, 0.78);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   @Ignore
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testWalkingWithFootstepListWithLinePredictedSupportPolygon() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);

      for (FootstepDataListMessage footstepDataList : footstepDataLists)
      {
         drcSimulationTestHelper.send(footstepDataList);
         //      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
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
   public void testTakingStepsWithActualAndPredictedFootPolygonsChanging() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getSDFFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      
      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      
      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
      
      ArrayList<Point2d> contactPointsInAnkleFrame = generateContactPointsForSlightlyPulledInAnkleFrame(walkingControllerParameters, 1.0, 1.0);
      ArrayList<Point2d> contactPointsInSoleFrame = transformFromAnkleFrameToSoleFrame(contactPointsInAnkleFrame, ankleFrame, soleFrame);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FramePoint soleLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide));
      soleLocation.changeFrame(worldFrame);
      footstepData.setLocation(soleLocation.getPointCopy());
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footstepData.setPredictedContactPoints(contactPointsInSoleFrame);
      message.add(footstepData);
      
      
      drcSimulationTestHelper.send(message);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      
      
      
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }
   
   
   @Ignore
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testHoldPositionByStandingOnOneLegAndGettingPushedSideways() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setUsePefectSensors(true);

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
      
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

//
//      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

//      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);
//
//      for (FootstepDataListMessage footstepDataList : footstepDataLists)
//      {
//         drcSimulationTestHelper.send(footstepDataList);
//         //      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));
//
//         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
//      }
//
//      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
//      drcSimulationTestHelper.checkNothingChanged();
//
//      assertTrue(success);
//
//      Point3d center = new Point3d(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
//      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
//      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
//      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   private void changeAppendageGroundContactPointsToNewOffsets(SDFHumanoidRobot robot, ArrayList<Point2d> newContactPoints, String jointName)
   {
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
   
   
   
   @Ignore
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testStandingWithLeftFootContactsChangeToFrontOnly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

      ArrayList<Point2d> newContactPoints = generateContactPointsForFrontOfFoot(getRobotModel().getWalkingControllerParameters());

      int pointIndex = 0;

      for (GroundContactPoint point : allGroundContactPoints)
      {
         Joint parentJoint = point.getParentJoint();

         if (parentJoint.getName().equals("l_leg_akx"))
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

      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

//      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);
//
//      for (FootstepDataListMessage footstepDataList : footstepDataLists)
//      {
//         drcSimulationTestHelper.send(footstepDataList);
//         //      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));
//
//         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
//      }
//
//      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
//      drcSimulationTestHelper.checkNothingChanged();
//
//      assertTrue(success);
//
//      Point3d center = new Point3d(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
//      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
//      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
//      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }
   
   
   @Ignore
   @DeployableTestMethod(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testWalkingWithGCPointsChangingOnTheFly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      Random random = new Random(1984L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SDFHumanoidRobot robot = drcSimulationTestHelper.getRobot();
      ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

      ArrayList<Point2d> newContactPoints = generatePredictedContactPoints(random);

      int pointIndex = 0;

      for (GroundContactPoint point : allGroundContactPoints)
      {
         Joint parentJoint = point.getParentJoint();
         System.out.println(parentJoint.getName());

         if (parentJoint.getName().equals("l_leg_akx"))
         {
            Point2d newContactPoint = newContactPoints.get(pointIndex);

            point.setIsInContact(false);
            Vector3d offset = new Vector3d();
            point.getOffset(offset);
//            offset.scale(0.5);
            offset.setX(newContactPoint.getX() + 0.05);
            offset.setY(newContactPoint.getY());

            point.setOffsetJoint(offset);
         }  
      }

      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

//      ArrayList<FootstepDataListMessage> footstepDataLists = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);
//
//      for (FootstepDataListMessage footstepDataList : footstepDataLists)
//      {
//         drcSimulationTestHelper.send(footstepDataList);
//         //      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));
//
//         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
//      }
//
//      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
//      drcSimulationTestHelper.checkNothingChanged();
//
//      assertTrue(success);
//
//      Point3d center = new Point3d(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
//      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
//      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
//      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<FootstepDataListMessage> createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
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
      
      ret.add(new Point2d(footForwardOffset, toeWidth/2.0));
      ret.add(new Point2d(footForwardOffset, -toeWidth/2.0));
      ret.add(new Point2d(-footBackwardOffset, -footWidth/2.0));
      ret.add(new Point2d(-footBackwardOffset, footWidth/2.0));
      return ret;
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

   
   private ArrayList<Point2d> generateContactPointsForFrontOfFoot(WalkingControllerParameters walkingControllerParameters)
   {
      double footLength = walkingControllerParameters.getFootLength();
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      
      ArrayList<Point2d> ret = new ArrayList<Point2d>();
//      ret.add(new Point2d(-footBackwardOffset, footWidth/2.0));
//      ret.add(new Point2d(-footBackwardOffset, -footWidth/2.0));
      
      ret.add(new Point2d(footForwardOffset/2.0, footWidth/2.0));
      ret.add(new Point2d(footForwardOffset/2.0, -footWidth/2.0));
      ret.add(new Point2d(footForwardOffset, -footWidth/2.0));
      ret.add(new Point2d(footForwardOffset, footWidth/2.0));
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

   private FootstepDataListMessage createFootstepsForWalkingOnFlatLongStepsOld(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {
            { { 0.5909646234016005, 0.10243127081250579, 0.08400000000000002 }, { 3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856 } },
            { { 1.212701966120992, -0.09394691394679651, 0.084 }, { 1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856 } },
            { { 1.8317941784239657, 0.11014657591704705, 0.08619322927296164 }, { 8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856 } },
            { { 2.4535283480857237, -0.08575120920059497, 0.08069788195751608 }, { -2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856 } },
            { { 3.073148474156348, 0.11833676240086898, 0.08590468550531082 }, { 4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056 } },
            { { 3.0729346702590505, -0.0816428320664241, 0.0812390388356 }, { -8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641 } } };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   protected abstract Vector3d getFootSlipVector();

   protected abstract double getFootSlipTimeDeltaAfterTouchdown();

   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);
}
