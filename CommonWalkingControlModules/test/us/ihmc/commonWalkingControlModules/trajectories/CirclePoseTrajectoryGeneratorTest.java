package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameOrientationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVectorTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

/**
 * @author twan
 *         Date: 6/12/13
 */
public class CirclePoseTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-12;
   private ReferenceFrame worldFrame;
   private Random random = new Random(12525L);
   private YoVariableRegistry registry;
   private OrientationProvider initialOrientationProvider;
   private DoubleProvider trajectoryTimeProvider;
   private CirclePoseTrajectoryGenerator trajectoryGenerator;

   @Before
   public void setUp()
   {
      worldFrame = ReferenceFrame.getWorldFrame();
      registry = new YoVariableRegistry("reg");

      FramePose initialPose = FramePose.generateRandomFramePose(random, worldFrame, 1.0, 1.0, 1.0);
      FrameOrientation initialOrientation = new FrameOrientation();
      initialPose.getOrientationIncludingFrame(initialOrientation);
      initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);

      trajectoryTimeProvider = new ConstantDoubleProvider(1.0);

      trajectoryGenerator = new CirclePoseTrajectoryGenerator("test", worldFrame, trajectoryTimeProvider, registry, null);
      trajectoryGenerator.setDesiredRotationAngle(Math.PI);
      trajectoryGenerator.setInitialPose(initialPose);
      trajectoryGenerator.initialize();
   }

   @After
   public void tearDown()
   {
      worldFrame = null;
      random = null;
      registry = null;
      initialOrientationProvider = null;
      trajectoryTimeProvider = null;
      trajectoryGenerator = null;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrientation()
   {
      trajectoryGenerator.setControlHandAngleAboutAxis(true);

      // v = omega x r
      checkVEqualsOmegaCrossR(worldFrame, trajectoryGenerator, random);

      checkOrientationAtVariousPoints(trajectoryGenerator, initialOrientationProvider, trajectoryTimeProvider.getValue(), worldFrame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCompute()
   {
      trajectoryGenerator.setControlHandAngleAboutAxis(true);
      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() * 0.5);

      FrameVector velocityToPack = new FrameVector(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertTrue(velocityToPack.getZ() != 0.0);
      FrameVector accelerationToPack = new FrameVector(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularAcceleration(accelerationToPack);
      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      //      assertTrue(accelerationToPack.getZ() != 0.0); // Not the case anymore as we've switched to cubic spline

      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() + 1.0);

      velocityToPack = new FrameVector(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      accelerationToPack = new FrameVector(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularAcceleration(accelerationToPack);
      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   //TODO: implement a real test
   public void testGetPosition()
   {
      trajectoryGenerator.compute(0.0);
      FramePoint currentPosition = new FramePoint();
      trajectoryGenerator.getPosition(currentPosition);
      currentPosition.getX();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsDone()
   {
      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() / 2.0);
      assertFalse(trajectoryGenerator.isDone());

      trajectoryGenerator.compute(trajectoryTimeProvider.getValue());
      assertTrue(trajectoryGenerator.isDone());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGet_FramePoint()
   {
      FramePoint positionToPack = new FramePoint();

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGet_FrameOrientation()
   {
      FrameOrientation orientationToPack = new FrameOrientation();

      trajectoryGenerator.getOrientation(orientationToPack);

      assertEquals(worldFrame, orientationToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackVelocity()
   {
      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(velocityToPack.getReferenceFrame()));

      trajectoryGenerator.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(worldFrame, velocityToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackAcceleration()
   {
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(accelerationToPack.getReferenceFrame()));

      trajectoryGenerator.getAcceleration(accelerationToPack);

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(worldFrame, accelerationToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackAngularVelocity()
   {
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(angularVelocityToPack.getReferenceFrame()));

      trajectoryGenerator.getVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(worldFrame, angularVelocityToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackAngularAcceleration()
   {
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      trajectoryGenerator.getAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(worldFrame, angularAccelerationToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackLinearData()
   {
      FramePoint positionToPack = new FramePoint(worldFrame);
      positionToPack.setIncludingFrame(worldFrame, 4.4, 3.3, 1.4);

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(velocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(worldFrame.equals(accelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

      trajectoryGenerator.getLinearData(positionToPack, velocityToPack, accelerationToPack);

      assertEquals(0.0, positionToPack.getX(), EPSILON);
      assertEquals(0.0, positionToPack.getY(), EPSILON);
      assertEquals(0.0, positionToPack.getZ(), EPSILON);
      assertSame(worldFrame, positionToPack.getReferenceFrame());

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(worldFrame, velocityToPack.getReferenceFrame());

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(worldFrame, accelerationToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackAngularData()
   {
      FramePoint positionToPack = new FramePoint(worldFrame);
      positionToPack.setIncludingFrame(worldFrame, 4.4, 3.3, 1.4);

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(worldFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));

      assertFalse(worldFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));

      trajectoryGenerator.getLinearData(positionToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, positionToPack.getX(), EPSILON);
      assertEquals(0.0, positionToPack.getY(), EPSILON);
      assertEquals(0.0, positionToPack.getZ(), EPSILON);
      assertSame(worldFrame, positionToPack.getReferenceFrame());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(worldFrame, angularVelocityToPack.getReferenceFrame());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(worldFrame, angularAccelerationToPack.getReferenceFrame());
   }

   private void checkOrientationAtVariousPoints(CirclePoseTrajectoryGenerator trajectoryGenerator, OrientationProvider initialOrientationProvider, double tMax,
         ReferenceFrame frame)
   {
      FrameOrientation orientation = new FrameOrientation(frame);

      trajectoryGenerator.compute(0.0);
      trajectoryGenerator.getOrientation(orientation);

      FrameOrientation initialOrientation = new FrameOrientation(frame);
      initialOrientationProvider.getOrientation(initialOrientation);

      FrameOrientationTest.assertFrameOrientationEquals(initialOrientation, orientation, 1e-12);

      FramePoint initialPosition = new FramePoint(frame);
      trajectoryGenerator.getPosition(initialPosition);

      FramePoint newPosition = new FramePoint(frame);

      /*
       * check against rotated version of initial position
       */
      int nTests = 10;
      for (int i = 1; i < nTests; i++)
      {
         double t = i * (tMax / nTests);
         trajectoryGenerator.compute(t);

         trajectoryGenerator.getPosition(newPosition);
         trajectoryGenerator.getOrientation(orientation);

         AxisAngle4d difference = FrameOrientationTest.computeDifferenceAxisAngle(initialOrientation, orientation);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(difference);

         FramePoint rotatedInitialPosition = new FramePoint(initialPosition);
         rotationMatrix.transform(rotatedInitialPosition.getPoint());
         //         JUnitTools.assertFramePointEquals(newPosition, rotatedInitialPosition, 1e-9);
      }
   }

   private void checkVEqualsOmegaCrossR(ReferenceFrame frame, CirclePoseTrajectoryGenerator trajectoryGenerator, Random random)
   {
      trajectoryGenerator.compute(random.nextDouble());

      FrameVector omega = new FrameVector(frame);
      trajectoryGenerator.getAngularVelocity(omega);

      FrameVector v = new FrameVector(frame);
      trajectoryGenerator.getVelocity(v);

      FramePoint r = new FramePoint(frame);
      trajectoryGenerator.getPosition(r);

      FrameVector vCheck = new FrameVector(frame);
      vCheck.cross(omega, r);

      FrameVectorTest.assertFrameVectorEquals(vCheck, v, 1e-8);
   }
}
