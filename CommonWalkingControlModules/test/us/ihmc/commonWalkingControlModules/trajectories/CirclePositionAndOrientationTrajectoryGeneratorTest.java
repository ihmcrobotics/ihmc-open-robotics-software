package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.utilities.math.trajectories.providers.ConstantPositionProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.OrientationProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.utilities.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.trajectories.CirclePositionAndOrientationTrajectoryGenerator;


/**
 * @author twan
 *         Date: 6/12/13
 */
public class CirclePositionAndOrientationTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-12;
   private static final boolean DEBUG = false;
   private ReferenceFrame frame;
   private Random random = new Random(12525L);
   private YoVariableRegistry registry;
   private AxisAngle4d initialRotationFromWorld;
   private Matrix3d initialRotationFromWorldMatrix;
   private OrientationProvider initialOrientationProvider;
   private PositionProvider initialPositionProvider;
   private DoubleProvider desiredRotationAngleProvider;
   private DoubleProvider trajectoryTimeProvider;
   private CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator;

   @Before
   public void setUp()
   {
      frame = ReferenceFrame.getWorldFrame();
      registry = new YoVariableRegistry("reg");
      initialPositionProvider = new ConstantPositionProvider(new FramePoint(frame, RandomTools.generateRandomVector(random)));

      initialRotationFromWorld = RandomTools.generateRandomRotation(random);
      initialRotationFromWorldMatrix = new Matrix3d();
      initialRotationFromWorldMatrix.set(initialRotationFromWorld);
      initialOrientationProvider = new ConstantOrientationProvider(new FrameOrientation(frame, initialRotationFromWorldMatrix));
      
      desiredRotationAngleProvider = new ConstantDoubleProvider(Math.PI);
      trajectoryTimeProvider = new ConstantDoubleProvider(1.0);
      
      trajectoryGenerator = new CirclePositionAndOrientationTrajectoryGenerator("test", trajectoryTimeProvider, initialOrientationProvider,
            initialPositionProvider, registry, desiredRotationAngleProvider, null);
      trajectoryGenerator.setCircleReferenceFrame(frame);
      trajectoryGenerator.initialize();
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testOrientation()
   {
      // v = omega x r
      checkVEqualsOmegaCrossR(frame, trajectoryGenerator, random);

      checkOrientationAtVariousPoints(trajectoryGenerator, initialOrientationProvider, trajectoryTimeProvider.getValue(), frame);
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testCompute()
   {
      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() * 0.5);

      FrameVector velocityToPack = new FrameVector(frame, 1.1, 2.2, 3.3);
      trajectoryGenerator.packAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertTrue(velocityToPack.getZ() != 0.0);
      FrameVector accelerationToPack = new FrameVector(frame, 1.1, 2.2, 3.3);
      trajectoryGenerator.packAngularAcceleration(accelerationToPack);
      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertTrue(accelerationToPack.getZ() != 0.0);

      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() + 1.0);

      velocityToPack = new FrameVector(frame, 1.1, 2.2, 3.3);
      trajectoryGenerator.packAngularVelocity(velocityToPack);
      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      accelerationToPack = new FrameVector(frame, 1.1, 2.2, 3.3);
      trajectoryGenerator.packAngularAcceleration(accelerationToPack);
      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
   }

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   //TODO: implement a real test
   public void testGetPosition()
   {
      trajectoryGenerator.compute(0.0);
      trajectoryGenerator.getPosition().getX();
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testIsDone()
   {
      trajectoryGenerator.compute(trajectoryTimeProvider.getValue() / 2.0);
      assertFalse(trajectoryGenerator.isDone());

      trajectoryGenerator.compute(trajectoryTimeProvider.getValue());
      assertTrue(trajectoryGenerator.isDone());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testGet_FramePoint()
   {
      FramePoint positionToPack = new FramePoint();

      trajectoryGenerator.get(positionToPack);

      assertEquals(frame, positionToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testGet_FrameOrientation()
   {
      FrameOrientation orientationToPack = new FrameOrientation();

      trajectoryGenerator.get(orientationToPack);

      assertEquals(frame, orientationToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackVelocity()
   {
      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(velocityToPack.getReferenceFrame()));

      trajectoryGenerator.packVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(frame, velocityToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackAcceleration()
   {
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(accelerationToPack.getReferenceFrame()));

      trajectoryGenerator.packAcceleration(accelerationToPack);

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(frame, accelerationToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackAngularVelocity()
   {
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(angularVelocityToPack.getReferenceFrame()));

      trajectoryGenerator.packVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(frame, angularVelocityToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackAngularAcceleration()
   {
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(angularAccelerationToPack.getReferenceFrame()));

      trajectoryGenerator.packAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(frame, angularAccelerationToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackLinearData()
   {
      FramePoint positionToPack = new FramePoint(frame);
      positionToPack.setIncludingFrame(frame, 4.4, 3.3, 1.4);

      trajectoryGenerator.get(positionToPack);

      assertEquals(frame, positionToPack.getReferenceFrame());

      trajectoryGenerator.get(positionToPack);

      assertEquals(frame, positionToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(velocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(frame.equals(accelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

      trajectoryGenerator.packLinearData(positionToPack, velocityToPack, accelerationToPack);

      assertEquals(0.0, positionToPack.getX(), EPSILON);
      assertEquals(0.0, positionToPack.getY(), EPSILON);
      assertEquals(0.0, positionToPack.getZ(), EPSILON);
      assertSame(frame, positionToPack.getReferenceFrame());

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(frame, velocityToPack.getReferenceFrame());

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(frame, accelerationToPack.getReferenceFrame());
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testPackAngularData()
   {
      FramePoint positionToPack = new FramePoint(frame);
      positionToPack.setIncludingFrame(frame, 4.4, 3.3, 1.4);

      trajectoryGenerator.get(positionToPack);

      assertEquals(frame, positionToPack.getReferenceFrame());

      trajectoryGenerator.get(positionToPack);

      assertEquals(frame, positionToPack.getReferenceFrame());

      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(frame.equals(angularVelocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));

      assertFalse(frame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));

      trajectoryGenerator.packLinearData(positionToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, positionToPack.getX(), EPSILON);
      assertEquals(0.0, positionToPack.getY(), EPSILON);
      assertEquals(0.0, positionToPack.getZ(), EPSILON);
      assertSame(frame, positionToPack.getReferenceFrame());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(frame, angularVelocityToPack.getReferenceFrame());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(frame, angularAccelerationToPack.getReferenceFrame());
   }
   
   private void checkOrientationAtVariousPoints(CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator,
         OrientationProvider initialOrientationProvider, double tMax, ReferenceFrame frame)
   {
      FrameOrientation orientation = new FrameOrientation(frame);

      trajectoryGenerator.compute(0.0);
      trajectoryGenerator.get(orientation);

      FrameOrientation initialOrientation = new FrameOrientation(frame);
      initialOrientationProvider.get(initialOrientation);

      JUnitTools.assertFrameOrientationEquals(initialOrientation, orientation, 1e-12);

      FramePoint initialPosition = new FramePoint(frame);
      trajectoryGenerator.get(initialPosition);

      FramePoint newPosition = new FramePoint(frame);

      /*
       * check against rotated version of initial position
       */
      int nTests = 10;
      for (int i = 1; i < nTests; i++)
      {
         double t = i * (tMax / nTests);
         trajectoryGenerator.compute(t);

         trajectoryGenerator.get(newPosition);
         trajectoryGenerator.get(orientation);

         AxisAngle4d difference = JUnitTools.computeDifferenceAxisAngle(initialOrientation, orientation);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(difference);

         FramePoint rotatedInitialPosition = new FramePoint(initialPosition);
         rotationMatrix.transform(rotatedInitialPosition.getPoint());
//         JUnitTools.assertFramePointEquals(newPosition, rotatedInitialPosition, 1e-9);
      }
   }

   private void checkVEqualsOmegaCrossR(ReferenceFrame frame, CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator, Random random)
   {
      trajectoryGenerator.compute(random.nextDouble());

      FrameVector omega = new FrameVector(frame);
      trajectoryGenerator.packAngularVelocity(omega);

      FrameVector v = new FrameVector(frame);
      trajectoryGenerator.packVelocity(v);

      FramePoint r = new FramePoint(frame);
      trajectoryGenerator.get(r);

      FrameVector vCheck = new FrameVector(frame);
      vCheck.cross(omega, r);

      JUnitTools.assertFrameVectorEquals(vCheck, v, 1e-8);
   }
}