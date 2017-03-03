package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint3d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;
import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndFootTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 41.5)
   @Test(timeout = 210000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double trajectoryTime = 1.0;
         FramePose desiredRandomFootPose = new FramePose(foot.getBodyFixedFrame());
         desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
         desiredRandomFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
         desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         desiredRandomFootPose.getPose(desiredPosition, desiredOrientation);
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);


         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 46.8)
   @Test(timeout = 230000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         double timePerWaypoint = 0.1;
         int numberOfTrajectoryPoints = 25;
         double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(ankleFrame, 0.0, radiusY * Math.cos(angle), radiusZ * Math.sin(2.0 * angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            footTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
         }

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         assertNumberOfWaypoints(robotSide, numberOfTrajectoryPoints + 1, scs);

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage fromMessage = footTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + firstTrajectoryPointTime);
         assertTrue(success);

         SE3TrajectoryPointMessage fromMessage = footTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 68.1)
   @Test(timeout = 340000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 100;
         double trajectoryTime = 7.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint sphereCenter = new FramePoint(ankleFrame);
         sphereCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusXY = 0.25;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(1.0, numberOfTrajectoryPoints);

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            pointsOnSphere[i].setX(pointsOnSphere[i].getX() * radiusXY);
            pointsOnSphere[i].setY(pointsOnSphere[i].getY() * radiusXY);
            pointsOnSphere[i].setZ(pointsOnSphere[i].getZ() * radiusZ);
            tempPoint.setIncludingFrame(ankleFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            footTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
         }

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(firstTrajectoryPointTime + 4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         int expectedTrajectoryPointIndex = 0;
         boolean isDone = false;
         double previousTimeInState = firstTrajectoryPointTime;

         while(!isDone)
         {
            assertNumberOfWaypoints(robotSide, Math.min(defaultMaximumNumberOfWaypoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs);

            double timeInState = 0.0;

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < defaultMaximumNumberOfWaypoints - 1; trajectoryPointIndex++)
            {
               SE3TrajectoryPointMessage fromMessage = footTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex);
               SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
               expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
               SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
               assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

               timeInState = Math.max(fromMessage.time, timeInState);

               expectedTrajectoryPointIndex++;

               if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
               {
                  isDone = true;
                  break;
               }
            }

            double simulationTime = timeInState - previousTimeInState;
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
            assertTrue(success);
            previousTimeInState = timeInState;
         }

         SE3TrajectoryPointMessage fromMessage = footTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

            // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 108.9)
   @Test(timeout = 540000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.15);
         double radiusXY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         List<FootTrajectoryMessage> messages = new ArrayList<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
               if (robotSide == RobotSide.LEFT)
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot), radiusZ * Math.sin(2.0 * angle));
               else
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle), radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
               tempPoint.add(circleCenter);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
            }
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            footTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            messages.add(footTrajectoryMessage);
            drcSimulationTestHelper.send(footTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);

         double timeOffset = 0.0;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double simulationTime = 0.0;

            assertNumberOfWaypoints(robotSide, messages.get(messageIndex).getNumberOfTrajectoryPoints() + 1, scs);

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               SE3TrajectoryPointMessage fromMessage = messages.get(messageIndex).getTrajectoryPoint(trajectoryPointIndex);
               SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
               expectedTrajectoryPoint.set(fromMessage.time + timeOffset, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
               SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
               assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

               simulationTime = Math.max(fromMessage.time, simulationTime);
            }
            timeOffset += simulationTime;
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
            assertTrue(success);
         }

         SE3TrajectoryPointMessage fromMessage = messages.get(numberOfMessages - 1).getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 32.1)
   @Test(timeout = 160000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusXY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         List<FootTrajectoryMessage> messages = new ArrayList<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
               if (robotSide == RobotSide.LEFT)
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot), radiusZ * Math.sin(2.0 * angle));
               else
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle), radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
               tempPoint.add(circleCenter);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
            }
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            footTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
            }
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            messages.add(footTrajectoryMessage);
            drcSimulationTestHelper.send(footTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05 + getRobotModel().getControllerDT());
         assertTrue(success);

         assertNumberOfWaypoints(robotSide, 1, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 38.0)
   @Test(timeout = 190000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusXY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         List<FootTrajectoryMessage> messages = new ArrayList<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
               if (robotSide == RobotSide.LEFT)
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot), radiusZ * Math.sin(2.0 * angle));
               else
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle), radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
               tempPoint.add(circleCenter);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
            }
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            footTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            messages.add(footTrajectoryMessage);
            drcSimulationTestHelper.send(footTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);


         trajectoryTime = 1.0;
         FramePose desiredRandomFootPose = new FramePose(foot.getBodyFixedFrame());
         Random random = new Random(545L);
         desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
         desiredRandomFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
         desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         desiredRandomFootPose.getPose(desiredPosition, desiredOrientation);
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   public static Point3D findControllerDesiredPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      return findPoint3d(subTrajectoryName, currentPositionVarNamePrefix, scs);
   }

   public static Quaternion findControllerDesiredOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      return findQuat4d(subTrajectoryName, currentOrientationVarNamePrefix, scs);
   }

   public static Vector3D findControllerDesiredLinearVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentLinearVelocityVarNamePrefix = subTrajectoryName + "CurrentVelocity";

      return findVector3d(subTrajectoryName, currentLinearVelocityVarNamePrefix, scs);
   }

   public static Vector3D findControllerDesiredAngularVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentAngularVelocityVarNamePrefix = subTrajectoryName + "CurrentAngularVelocity";

      return findVector3d(subTrajectoryName, currentAngularVelocityVarNamePrefix, scs);
   }

   public static int findNumberOfWaypointsForOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String numberOfWaypointsVarName = footPrefix + "NumberOfWaypoints";
      String orientationTrajectoryName = footPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static int findNumberOfWaypointsForPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String numberOfWaypointsVarName = footPrefix + "NumberOfWaypoints";
      String positionTrajectoryName = footPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(RobotSide robotSide, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String positionTrajectoryName = footPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String orientationTrajectoryName = footPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = footPrefix + "Time";
      String positionName = footPrefix + "Position";
      String orientationName = footPrefix + "Orientation";
      String linearVelocityName = footPrefix + "LinearVelocity";
      String angularVelocityName = footPrefix + "AngularVelocity";

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setTime(scs.getVariable(positionTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(findPoint3d(positionTrajectoryName, positionName, suffix, scs));
      simpleSE3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findVector3d(positionTrajectoryName, linearVelocityName, suffix, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static SimpleSE3TrajectoryPoint findLastTrajectoryPoint(RobotSide robotSide, SimulationConstructionSet scs)
   {
      int numberOfWaypoints = findNumberOfWaypointsForPosition(robotSide, scs);
      return findTrajectoryPoint(robotSide, numberOfWaypoints - 1, scs);
   }

   public static SimpleSE3TrajectoryPoint findCurrentDesiredTrajectoryPoint(RobotSide robotSide, SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setPosition(findControllerDesiredPosition(robotSide, scs));
      simpleSE3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(robotSide, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findControllerDesiredLinearVelocity(robotSide, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(robotSide, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(RobotSide robotSide, Point3D desiredPosition, Quaternion desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(robotSide, 2, scs);

      Point3D controllerDesiredPosition = findControllerDesiredPosition(robotSide, scs);
      assertEquals(desiredPosition.getX(), controllerDesiredPosition.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), controllerDesiredPosition.getY(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getZ(), controllerDesiredPosition.getZ(), EPSILON_FOR_DESIREDS);

      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(robotSide, scs);
      assertEquals(desiredOrientation.getX(), controllerDesiredOrientation.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getY(), controllerDesiredOrientation.getY(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getZ(), controllerDesiredOrientation.getZ(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getS(), controllerDesiredOrientation.getS(), EPSILON_FOR_DESIREDS);
   }

   public static void assertNumberOfWaypoints(RobotSide robotSide, int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      assertEquals(expectedNumberOfTrajectoryPoints, findNumberOfWaypointsForPosition(robotSide, scs));
      assertEquals(expectedNumberOfTrajectoryPoints, findNumberOfWaypointsForOrientation(robotSide, scs));
   }

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
