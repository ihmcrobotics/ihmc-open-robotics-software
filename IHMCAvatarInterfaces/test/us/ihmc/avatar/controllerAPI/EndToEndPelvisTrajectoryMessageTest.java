package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint2d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint3d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;
import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndPelvisTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.2e-4;
   private static final double EPSILON_FOR_HEIGHT = 1.0e-2;

   private static final boolean DEBUG = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 16.9)
   @Test(timeout = 84000)
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

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePose desiredRandomPelvisPose = new FramePose(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.1);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      Point2D desiredPosition2d = new Point2D();
      desiredPosition2d.set(desiredPosition.getX(), desiredPosition.getY());
      desiredPosition2d.applyTransform(fromWorldToMidFeetZUpTransform);
      Quaternion desiredOrientationCorrected = new Quaternion(desiredOrientation);
      desiredOrientationCorrected.applyTransform(fromWorldToMidFeetZUpTransform);

      desiredPosition.setX(desiredPosition2d.getX());
      desiredPosition.setY(desiredPosition2d.getY());
      desiredOrientation.set(desiredOrientationCorrected);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 + trajectoryTime);
      assertTrue(success);

      assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
   }

   @ContinuousIntegrationTest(estimatedDuration = 14.3)
   @Test(timeout = 72000)
   public void testMultipleWaypoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 15;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint pelvisPosition = new FramePoint(pelvis.getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(numberOfTrajectoryPoints);

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);
      double radius = 0.1;
      double heightAmp = 0.1;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double pitch = amp * Math.sin(t * w);
         double pitchDot = w * amp * Math.cos(t * w);

         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         orientation.setYawPitchRoll(0.0, pitch, 0.0);
         angularVelocity.set(0.0, pitchDot, 0.0);

         double percent = trajectoryPointIndex / (numberOfTrajectoryPoints - 1.0);
         double angle = percent * 2.0 * Math.PI;
         double x = percent * radius * Math.cos(angle) + pelvisPosition.getX();
         double y = percent * radius * Math.sin(angle) + pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * angle) + pelvisPosition.getZ();

         double dx = - percent * radius * Math.sin(angle);
         double dy = percent * radius * Math.cos(angle);
         double dz = 2.0 * heightAmp * Math.cos(2.0 * angle);

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);

         pelvisTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, position, orientation, linearVelocity, angularVelocity);
      }

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBodyTransform fromWorldToDesiredPelvisOrientation = new RigidBodyTransform();
      Quaternion walkingDesiredOrientation = findQuat4d(PelvisOrientationManager.class.getSimpleName(), "desiredPelvis", scs);
      walkingDesiredOrientation.conjugate();
      fromWorldToDesiredPelvisOrientation.setRotation(walkingDesiredOrientation);

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(numberOfTrajectoryPoints + 1, scs);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);


         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         // Not checking the height on purpose as it is non-trivial.
         assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), EPSILON_FOR_DESIREDS);

         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         expectedTrajectoryPoint.applyTransform(fromWorldToDesiredPelvisOrientation);

         EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getLastTrajectoryPoint();
      SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
      expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
      SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);

      // Not check the height on purpose as it is non-trivial.
      assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
      assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), EPSILON_FOR_DESIREDS);

      expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
      expectedTrajectoryPoint.applyTransform(fromWorldToDesiredPelvisOrientation);

      EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
   }

   @ContinuousIntegrationTest(estimatedDuration = 31.7)
   @Test(timeout = 160000)
   public void testMessageWithTooManyWaypoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint pelvisPosition = new FramePoint(pelvis.getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(numberOfTrajectoryPoints);

      double t = 0.0;
      double w = 4.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);
      double radius = 0.1;
      double heightAmp = 0.1;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double pitch = amp * Math.sin(t * w);
         double pitchDot = w * amp * Math.cos(t * w);

         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         orientation.setYawPitchRoll(0.0, pitch, 0.0);
         angularVelocity.set(0.0, pitchDot, 0.0);

         double percent = trajectoryPointIndex / (numberOfTrajectoryPoints - 1.0);
         double angle = percent * 6.0 * Math.PI;
         double x = percent * radius * Math.cos(angle) + pelvisPosition.getX();
         double y = percent * radius * Math.sin(angle) + pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * angle) + pelvisPosition.getZ();

         double dx = - percent * radius * Math.sin(angle);
         double dy = percent * radius * Math.cos(angle);
         double dz = 2.0 * heightAmp * Math.cos(2.0 * angle);

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);

         pelvisTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, position, orientation, linearVelocity, angularVelocity);
      }

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBodyTransform fromWorldToDesiredPelvisOrientation = new RigidBodyTransform();
      Quaternion walkingDesiredOrientation = findQuat4d(PelvisOrientationManager.class.getSimpleName(), "desiredPelvis", scs);
      walkingDesiredOrientation.conjugate();
      fromWorldToDesiredPelvisOrientation.setRotation(walkingDesiredOrientation);

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint + 2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      int expectedTrajectoryPointIndex = 0;
      boolean isDone = false;
      double previousTimeInState = timePerWaypoint;

      while(!isDone)
      {
         assertNumberOfWaypoints(Math.min(defaultMaximumNumberOfWaypoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs);

         double timeInState = 0.0;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < defaultMaximumNumberOfWaypoints - 1; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
            expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);

            double time = expectedTrajectoryPoint.getTime();

            assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            // Not checking the height on purpose as it is non-trivial.
            assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
            EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), EPSILON_FOR_DESIREDS);

            expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
            expectedTrajectoryPoint.applyTransform(fromWorldToDesiredPelvisOrientation);

            EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
            EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

            timeInState = Math.max(time, timeInState);

            expectedTrajectoryPointIndex++;

            if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
            {
               isDone = true;
               break;
            }
         }

         double simulationTime = timeInState - previousTimeInState;
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime + getRobotModel().getControllerDT());
         assertTrue(success);
         previousTimeInState = timeInState;
         if (!isDone)
         {
            walkingDesiredOrientation = findQuat4d(PelvisOrientationManager.class.getSimpleName(), "desiredPelvis", scs);
            walkingDesiredOrientation.conjugate();
            fromWorldToDesiredPelvisOrientation.setRotation(walkingDesiredOrientation);

            midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
            midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
            midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
            midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
            fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
            fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
            fromWorldToMidFeetZUpTransform.invert();
         }
      }


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getLastTrajectoryPoint();
      SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
      expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
      SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);

      // Not checking the height on purpose as it is non-trivial.
      assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
      assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), EPSILON_FOR_DESIREDS);

      expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
      expectedTrajectoryPoint.applyTransform(fromWorldToDesiredPelvisOrientation);

      // These asserts are causing trouble
      EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
   }

   @ContinuousIntegrationTest(estimatedDuration = 16.5)
   @Test(timeout = 83000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePose desiredRandomPelvisPose = new FramePose(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.1);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAllTrajectoryMessage);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertFalse(findControllerStopBooleanForOrientation(scs));
      assertFalse(findControllerStopBooleanForXY(scs));
      assertFalse(findControllerStopBooleanForHeight(scs));
      Quaternion controllerDesiredOrientationBeforeStop = findControllerDesiredOrientation(scs);
      Point2D controllerDesiredXYBeforeStop = findControllerDesiredPositionXY(scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      assertTrue(findControllerStopBooleanForOrientation(scs));
      assertTrue(findControllerStopBooleanForXY(scs));
      assertTrue(findControllerStopBooleanForHeight(scs));
      Quaternion controllerDesiredOrientationAfterStop = findControllerDesiredOrientation(scs);
      Point2D controllerDesiredXYAfterStop = findControllerDesiredPositionXY(scs);

      EuclidCoreTestTools.assertQuaternionEquals(controllerDesiredOrientationBeforeStop, controllerDesiredOrientationAfterStop, 1.0e-2);
      EuclidCoreTestTools.assertTuple2DEquals("", controllerDesiredXYBeforeStop, controllerDesiredXYAfterStop, 1.0e-2);
   }

   public static Quaternion findControllerDesiredOrientation(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      return findQuat4d(subTrajectoryName, currentOrientationVarNamePrefix, scs);
   }

   public static Point2D findControllerDesiredPositionXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      return findPoint2d(subTrajectoryName, currentPositionVarNamePrefix, scs);
   }

   public static double findCurrentPelvisHeight(SimulationConstructionSet scs)
   {
      return scs.getVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
   }

   public static Vector3D findControllerDesiredLinearVelocity(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectory = "SubTrajectory";
      String subTrajectoryName = pelvisPrefix + subTrajectory;
      String currentLinearVelocityVarNamePrefix = subTrajectoryName + "CurrentVelocity";

      Vector3D linearVelocity = findVector3d(subTrajectoryName, currentLinearVelocityVarNamePrefix, scs);

      String pelvisHeightPrefix = "pelvisHeightOffset";
      String offsetHeightTrajectoryName = pelvisHeightPrefix + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();

      linearVelocity.setZ(scs.getVariable(offsetHeightTrajectoryName, pelvisHeightPrefix + subTrajectory + "CurrentVelocity").getValueAsDouble());

      return linearVelocity;
   }

   public static Vector3D findControllerDesiredAngularVelocity(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentAngularVelocityVarNamePrefix = subTrajectoryName + "CurrentAngularVelocity";

      return findVector3d(subTrajectoryName, currentAngularVelocityVarNamePrefix, scs);
   }

   public static boolean findControllerStopBooleanForOrientation(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(PelvisOrientationManager.class.getSimpleName(), "isPelvisOrientationOffsetTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForXY(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(PelvisICPBasedTranslationManager.class.getSimpleName(), "isPelvisTranslationalTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForHeight(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(LookAheadCoMHeightTrajectoryGenerator.class.getSimpleName(), "isPelvisOffsetHeightTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypointsForOrientation(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String orientationTrajectoryName = pelvisPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsForXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String positionTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsForHeight(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisHeightOffset";
      String offsetHeightTrajectoryName = pelvisPrefix + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(offsetHeightTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   /** The z component of the position is set to {@linkplain Double#NaN} as it cannot be directly nor easily obtained. */
   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      String suffix = "AtWaypoint" + trajectoryPointIndex;

      {
         String pelvisPrefix = "pelvisOffset";
         String orientationTrajectoryName = pelvisPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
         String timeName = pelvisPrefix + "Time";
         String orientationName = pelvisPrefix + "Orientation";
         String angularVelocityName = pelvisPrefix + "AngularVelocity";

         simpleSE3TrajectoryPoint.setTime(scs.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
         simpleSE3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
         simpleSE3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      }

      {
         String pelvisXYPrefix = "pelvisOffset";
         String positionXYTrajectoryName = pelvisXYPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
         String positionXYName = pelvisXYPrefix + "Position";
         String linearVelocityXYName = pelvisXYPrefix + "LinearVelocity";

         String pelvisZPrefix = "pelvisHeightOffset";
         String positionZTrajectoryName = pelvisZPrefix + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
         String linearVelocityZName = pelvisZPrefix + "Velocity";

         Point3D position = findPoint3d(positionXYTrajectoryName, positionXYName, suffix, scs);
         position.setZ(Double.NaN);

         Vector3D linearVelocity = findVector3d(positionXYTrajectoryName, linearVelocityXYName, suffix, scs);
         linearVelocity.setZ(scs.getVariable(positionZTrajectoryName, linearVelocityZName + suffix).getValueAsDouble());

         simpleSE3TrajectoryPoint.setPosition(position);
         simpleSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      }

      return simpleSE3TrajectoryPoint;
   }

   public static SimpleSE3TrajectoryPoint findCurrentDesiredTrajectoryPoint(SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      Point2D positionXY = findControllerDesiredPositionXY(scs);
      Point3D position = new Point3D(positionXY.getX(), positionXY.getY(), Double.NaN);
      simpleSE3TrajectoryPoint.setPosition(position);
      simpleSE3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findControllerDesiredLinearVelocity(scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(scs));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(Point3D desiredPosition, Quaternion desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(2, scs);

      Point2D desiredControllerXY = findControllerDesiredPositionXY(scs);
      assertEquals(desiredPosition.getX(), desiredControllerXY.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), desiredControllerXY.getY(), EPSILON_FOR_DESIREDS);

      Quaternion desiredControllerOrientation = findControllerDesiredOrientation(scs);
      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientation, desiredControllerOrientation, EPSILON_FOR_DESIREDS);

      // Hard to figure out how to verify the desired there
//      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
//      assertEquals(desiredPosition.getZ(), trajOutput, EPSILON_FOR_DESIREDS);
      // Ending up doing a rough check on the actual height
      double pelvisHeight = findCurrentPelvisHeight(scs);
      assertEquals(desiredPosition.getZ(), pelvisHeight, EPSILON_FOR_HEIGHT);
   }

   public static void assertNumberOfWaypoints(int expectedNumberOfWaypoints, SimulationConstructionSet scs)
   {
      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForOrientation(scs));
      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForXY(scs));
      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForHeight(scs));
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
