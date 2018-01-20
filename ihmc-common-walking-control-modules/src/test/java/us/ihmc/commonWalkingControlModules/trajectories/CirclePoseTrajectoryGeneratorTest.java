package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

      FramePose3D initialPose = EuclidFrameRandomTools.nextFramePose3D(random, worldFrame, 1.0, 1.0, 1.0);
      FrameQuaternion initialOrientation = new FrameQuaternion(initialPose.getOrientation());
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

      FrameVector3D velocityToPack = new FrameVector3D(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertTrue(velocityToPack.getZ() != 0.0);
      FrameVector3D accelerationToPack = new FrameVector3D(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularAcceleration(accelerationToPack);
      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      //      assertTrue(accelerationToPack.getZ() != 0.0); // Not the case anymore as we've switched to cubic spline

      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() + 1.0);

      velocityToPack = new FrameVector3D(worldFrame, 1.1, 2.2, 3.3);
      trajectoryGenerator.getAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      accelerationToPack = new FrameVector3D(worldFrame, 1.1, 2.2, 3.3);
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
      FramePoint3D currentPosition = new FramePoint3D();
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
      FramePoint3D positionToPack = new FramePoint3D();

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGet_FrameOrientation()
   {
      FrameQuaternion orientationToPack = new FrameQuaternion();

      trajectoryGenerator.getOrientation(orientationToPack);

      assertEquals(worldFrame, orientationToPack.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackVelocity()
   {
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FramePoint3D positionToPack = new FramePoint3D(worldFrame);
      positionToPack.setIncludingFrame(worldFrame, 4.4, 3.3, 1.4);

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FramePoint3D positionToPack = new FramePoint3D(worldFrame);
      positionToPack.setIncludingFrame(worldFrame, 4.4, 3.3, 1.4);

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      trajectoryGenerator.getPosition(positionToPack);

      assertEquals(worldFrame, positionToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameQuaternion orientation = new FrameQuaternion(frame);

      trajectoryGenerator.compute(0.0);
      trajectoryGenerator.getOrientation(orientation);

      FrameQuaternion initialOrientation = new FrameQuaternion(frame);
      initialOrientationProvider.getOrientation(initialOrientation);

      EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(initialOrientation, orientation, 1e-12);

      FramePoint3D initialPosition = new FramePoint3D(frame);
      trajectoryGenerator.getPosition(initialPosition);

      FramePoint3D newPosition = new FramePoint3D(frame);

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

         FrameQuaternion difference = new FrameQuaternion(initialOrientation.getReferenceFrame());
         difference.difference(initialOrientation, orientation);

         FramePoint3D rotatedInitialPosition = new FramePoint3D(initialPosition);
         difference.transform(rotatedInitialPosition);
         //         JUnitTools.assertFramePointEquals(newPosition, rotatedInitialPosition, 1e-9);
      }
   }

   private void checkVEqualsOmegaCrossR(ReferenceFrame frame, CirclePoseTrajectoryGenerator trajectoryGenerator, Random random)
   {
      trajectoryGenerator.compute(random.nextDouble());

      FrameVector3D omega = new FrameVector3D(frame);
      trajectoryGenerator.getAngularVelocity(omega);

      FrameVector3D v = new FrameVector3D(frame);
      trajectoryGenerator.getVelocity(v);

      FramePoint3D r = new FramePoint3D(frame);
      trajectoryGenerator.getPosition(r);

      FrameVector3D vCheck = new FrameVector3D(frame);
      vCheck.cross(omega, r);

      EuclidFrameTestTools.assertFrameTuple3DEquals(vCheck, v, 1e-8);
   }
}
