package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

public class SpatialAccelerationVectorTest extends SpatialMotionVectorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 2.6)
	@Test(timeout = 30000)
   public void testChangeFrameUsingNumericalDifferentiationVersusAnalytical()
   {
      double epsilon = 1e-3;    // needs to be pretty high, but if you decrease deltaT, you can go lower
      double deltaT = 1e-6;

      double[] linearAmplitudes = {1.0, 2.0, 3.0};
      double[] angularAmplitudes = {4.0, 5.0, 6.0};
      double[] linearFrequencies = {1.0, 2.0, 3.0};
      double[] angularFrequencies = {4.0, 5.0, 6.0};

      Vector3D previousLinearVelocity = new Vector3D();
      Vector3D previousAngularVelocity = new Vector3D();
      double tMax = 1.0;

      for (double t = 0.0; t < tMax; t += deltaT)
      {
         Vector3D linearVelocity = getSinusoidalVelocity(linearAmplitudes, linearFrequencies, t);
         Vector3D angularVelocity = getSinusoidalVelocity(angularAmplitudes, angularFrequencies, t);

         Vector3D linearAcceleration = getSinusoidalAcceleration(linearAmplitudes, linearFrequencies, t);
         Vector3D angularAcceleration = getSinusoidalAcceleration(angularAmplitudes, angularFrequencies, t);

         Twist twistInB = new Twist(frameB, frameA, frameB, linearVelocity, angularVelocity);
         Twist twistInA = new Twist(twistInB);
         twistInA.changeFrame(frameA);

         SpatialAccelerationVector acceleration = new SpatialAccelerationVector(frameB, frameA, frameB, linearAcceleration, angularAcceleration);
         acceleration.changeFrame(frameA, twistInB, twistInB);

         if (t > deltaT / 2.0)    // numerically differentiating, so don't do the first step
         {
            Vector3D linearAccelerationNewFrameNumeric = numericallyDifferentiate(previousLinearVelocity, twistInA.getLinearPartCopy(), deltaT);
            Vector3D angularAccelerationNewFrameNumeric = numericallyDifferentiate(previousAngularVelocity, twistInA.getAngularPartCopy(), deltaT);

            Vector3D linearAccelerationNewFrameAnalytic = acceleration.getLinearPartCopy();
            Vector3D angularAccelerationNewFrameAnalytic = acceleration.getAngularPartCopy();

            EuclidCoreTestTools.assertTuple3DEquals("t = " + t, linearAccelerationNewFrameNumeric, linearAccelerationNewFrameAnalytic, epsilon);
            EuclidCoreTestTools.assertTuple3DEquals("t = " + t, angularAccelerationNewFrameNumeric, angularAccelerationNewFrameAnalytic, epsilon);
         }

         previousLinearVelocity.set(twistInA.getLinearPartCopy());
         previousAngularVelocity.set(twistInA.getAngularPartCopy());
      }
   }

   /**
    * Tests centripetal acceleration
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAccelerationOfPointFixedInBodyFrame()
   {
      Random random = new Random(1456L);

      SpatialAccelerationVector accel = new SpatialAccelerationVector(frameB, frameA, frameA, new Vector3D(), new Vector3D());    // zero relative acceleration
      Twist twist = new Twist(frameB, frameA, frameA, new Vector3D(), getRandomVector(random));    // pure rotational velocity
      FramePoint3D pointFixedInFrameB = new FramePoint3D(frameA, getRandomVector(random));
      FrameVector3D accelerationOfPointFixedInFrameB = new FrameVector3D(ReferenceFrame.getWorldFrame());
      accel.getAccelerationOfPointFixedInBodyFrame(twist, pointFixedInFrameB, accelerationOfPointFixedInFrameB);

      Vector3D expected = new Vector3D(pointFixedInFrameB);
      expected.cross(twist.getAngularPart(), expected);
      expected.cross(twist.getAngularPart(), expected);

      EuclidCoreTestTools.assertTuple3DEquals(expected, accelerationOfPointFixedInFrameB, 1e-7);
   }

	/**
	 * This test is used to prove that the reference frame in which the linear acceleration of a body fixed point in computed in does not matter.
	 */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetAccelerationOfPointFixedInBodyFrameComputedInDifferentFrames() throws Exception
   {
      Random random = new Random(345345L);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame baseFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("baseFrame", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));
         ReferenceFrame bodyFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("bodyFrame", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));
         
         Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3D angularAcceleration = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector(bodyFrame, baseFrame, bodyFrame, linearAcceleration, angularAcceleration);
         
         Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Twist twist = new Twist(bodyFrame, baseFrame, bodyFrame, linearVelocity, angularVelocity);
         
         FramePoint3D pointFixedInBodyFrame = new FramePoint3D(bodyFrame, EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         FrameVector3D bodyFixedPointLinearAccelerationInBody = new FrameVector3D();
         FrameVector3D bodyFixedPointLinearAccelerationInBase = new FrameVector3D();
         
         
         // Compute the linear acceleration while in bodyFrame
         pointFixedInBodyFrame.changeFrame(bodyFrame);
         spatialAccelerationVector.getAccelerationOfPointFixedInBodyFrame(twist, pointFixedInBodyFrame, bodyFixedPointLinearAccelerationInBody);
         
         // Compute the linear acceleration while in bodyFrame
         pointFixedInBodyFrame.changeFrame(baseFrame);
         spatialAccelerationVector.changeFrame(baseFrame, twist, twist);
         twist.changeFrame(baseFrame);
         spatialAccelerationVector.getAccelerationOfPointFixedInBodyFrame(twist, pointFixedInBodyFrame, bodyFixedPointLinearAccelerationInBase);
         
         // Verify that they are the same
         bodyFixedPointLinearAccelerationInBody.changeFrame(baseFrame);
         EuclidCoreTestTools.assertTuple3DEquals(bodyFixedPointLinearAccelerationInBase, bodyFixedPointLinearAccelerationInBody, 1.0e-12);
      }
   }

   // TODO: Figure out this test and get it to pass if it should.
	@Ignore
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testAccelerationOfPointFixedInBodyFrameAlternative()
   {
      Random random = new Random();
      SpatialAccelerationVector accel = new SpatialAccelerationVector(frameB, frameA, frameA, RandomGeometry.nextVector3D(random),
                                           RandomGeometry.nextVector3D(random));
      Twist twist = new Twist(frameB, frameA, frameA, RandomGeometry.nextVector3D(random), getRandomVector(random));
      FramePoint3D pointFixedInFrameB = new FramePoint3D(frameA);    // , getRandomVector(random));
      FrameVector3D accelerationOfPointFixedInFrameB = new FrameVector3D(ReferenceFrame.getWorldFrame());
      accel.getAccelerationOfPointFixedInBodyFrame(twist, pointFixedInFrameB, accelerationOfPointFixedInFrameB);

      Twist twistOfCurrentWithRespectToNew = new Twist(twist);
      twistOfCurrentWithRespectToNew.invert();
      Twist twistOfBodyWithRespectToBase = new Twist(twist);
      accel.changeFrame(frameB, twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);

      FrameVector3D expected = new FrameVector3D();
      accel.getLinearPart(expected);

      FrameVector3D crossPart = new FrameVector3D(expected.getReferenceFrame());
      twist.changeFrame(expected.getReferenceFrame());
      FrameVector3D omega = new FrameVector3D();
      twist.getAngularPart(omega);
      FrameVector3D v = new FrameVector3D();
      twist.getLinearPart(v);
      crossPart.cross(omega, v);
      expected.add(crossPart);
      expected.changeFrame(accelerationOfPointFixedInFrameB.getReferenceFrame());
      EuclidFrameTestTools.assertFrameTuple3DEquals(expected, accelerationOfPointFixedInFrameB, 1e-12);
   }


   /**
    * You shouldn't be able to add two spatial acceleration vectors expressed in different frames
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAddExpressedInDifferentFrames()
   {
      SpatialAccelerationVector acceleration1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
      SpatialAccelerationVector acceleration2 = createSpatialMotionVector(frameB, frameA, frameA, new Vector3D(), new Vector3D());

      acceleration1.add(acceleration2);
   }

   /**
    * You shouldn't be able to add two spatial acceleration vectors if the second is not relative to the first
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAddNotRelative()
   {
      SpatialAccelerationVector acceleration1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
      SpatialAccelerationVector acceleration2 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());

      acceleration1.add(acceleration2);
   }

   /**
    * Test adding two spatial motion vectors, both expressed in the same reference frame, and the second relative to the first
    * (which is allowed)
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAdd()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      SpatialAccelerationVector spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, linearVelocity1, angularVelocity1);

      Vector3D angularVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      SpatialAccelerationVector spatialMotionVector2 = createSpatialMotionVector(frameC, frameB, frameD, linearVelocity2, angularVelocity2);

      spatialMotionVector1.add(spatialMotionVector2);

      assertEquals(frameD, spatialMotionVector1.getExpressedInFrame());
      assertEquals(frameA, spatialMotionVector1.getBaseFrame());
      assertEquals(frameC, spatialMotionVector1.getBodyFrame());

      angularVelocity1.add(angularVelocity2);
      linearVelocity1.add(linearVelocity2);

      double epsilon = 1e-14;
      Vector3D angularVelocity1Plus2 = spatialMotionVector1.getAngularPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity1, angularVelocity1Plus2, epsilon);

      Vector3D linearVelocity1Plus2 = spatialMotionVector1.getLinearPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity1, linearVelocity1Plus2, epsilon);

      // Should throw exception if try it the other way:

      spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, linearVelocity1, angularVelocity1);
      spatialMotionVector2 = createSpatialMotionVector(frameC, frameB, frameD, linearVelocity2, angularVelocity2);

      try
      {
         spatialMotionVector2.add(spatialMotionVector1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }

      spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, linearVelocity1, angularVelocity1);

      try
      {
         spatialMotionVector1.add(spatialMotionVector1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSub()
   {
      Random random = new Random(3454L);
      SpatialAccelerationVector vector1 = new SpatialAccelerationVector(frameB, frameA, frameD, RandomGeometry.nextVector3D(random),
                                             RandomGeometry.nextVector3D(random));
      SpatialAccelerationVector vector2 = new SpatialAccelerationVector(frameC, frameB, frameD, RandomGeometry.nextVector3D(random),
                                             RandomGeometry.nextVector3D(random));
      SpatialAccelerationVector vector3 = new SpatialAccelerationVector(vector1);
      vector3.add(vector2);

      double epsilon = 1e-15;

      SpatialAccelerationVector vector2Back = new SpatialAccelerationVector(vector3);
      vector2Back.sub(vector1);
      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(vector2, vector2Back, epsilon);

      SpatialAccelerationVector vector1Back = new SpatialAccelerationVector(vector3);
      vector1Back.sub(vector2);
      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(vector1, vector1Back, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSubWrongExpressedInFrame()
   {
      SpatialAccelerationVector vector1 = new SpatialAccelerationVector(frameB, frameA, frameD);
      SpatialAccelerationVector vector2 = new SpatialAccelerationVector(frameB, frameC, frameC);
      vector1.sub(vector2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSubFramesDontMatchUp()
   {
      SpatialAccelerationVector vector1 = new SpatialAccelerationVector(frameD, frameA, frameC);
      SpatialAccelerationVector vector2 = new SpatialAccelerationVector(frameB, frameC, frameC);
      vector1.sub(vector2);
   }

   /**
    * Compares setScrew method in SpatialAccelerationVector to numerical derivative based method
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetScrew()
   {
      ReferenceFrame bodyFrame = frameA;
      ReferenceFrame baseFrame = frameB;
      ReferenceFrame expressedInFrame = frameC;

      double angularVelocityMagnitude = random.nextDouble();
      double angularAccelerationMagnitude = random.nextDouble();
      double linearVelocityMagnitude = random.nextDouble();
      double linearAccelerationMagnitude = random.nextDouble();
      Vector3D axisOfRotation = RandomGeometry.nextVector3D(random);
      axisOfRotation.normalize();
      Vector3D axisOfRotationDot = new Vector3D();
      axisOfRotationDot.cross(axisOfRotation, RandomGeometry.nextVector3D(random));
      Vector3D offset = RandomGeometry.nextVector3D(random);
      Vector3D offsetDot = RandomGeometry.nextVector3D(random);
      SpatialAccelerationVector acceleration = new SpatialAccelerationVector(bodyFrame, baseFrame, expressedInFrame, angularVelocityMagnitude,
                                                  angularAccelerationMagnitude, linearVelocityMagnitude, linearAccelerationMagnitude, axisOfRotation,
                                                  axisOfRotationDot, offset, offsetDot);

      Twist twist0 = new Twist();
      twist0.setScrew(bodyFrame, baseFrame, expressedInFrame, angularVelocityMagnitude, linearVelocityMagnitude, axisOfRotation, offset);

      double dt = 1e-8;
      angularVelocityMagnitude += angularAccelerationMagnitude * dt;
      linearVelocityMagnitude += linearAccelerationMagnitude * dt;
      Vector3D axisOfRotationDelta = new Vector3D(axisOfRotationDot);
      axisOfRotationDelta.scale(dt);
      axisOfRotation.add(axisOfRotationDelta);
      Vector3D offsetDelta = new Vector3D(offsetDot);
      offsetDelta.scale(dt);
      offset.add(offsetDelta);
      Twist twist1 = new Twist();
      twist1.setScrew(bodyFrame, baseFrame, expressedInFrame, angularVelocityMagnitude, linearVelocityMagnitude, axisOfRotation, offset);

      DenseMatrix64F numericalDerivative = new DenseMatrix64F(Twist.SIZE, 1);
      CommonOps.subtract(twist1.toMatrix(), twist0.toMatrix(), numericalDerivative);
      CommonOps.scale(1.0 / dt, numericalDerivative);

      JUnitTools.assertMatrixEquals(numericalDerivative, acceleration.toMatrix(), 1e-4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetBasedOnOriginAcceleration()
   {
      SpatialAccelerationVector acceleration = new SpatialAccelerationVector(frameA, frameB, frameA);
      Twist twistOfBodyWithRespectToBase = new Twist(frameA, frameB, frameA, RandomGeometry.nextVector3D(random),
                                              RandomGeometry.nextVector3D(random));
      FrameVector3D angularAcceleration = new FrameVector3D(twistOfBodyWithRespectToBase.getExpressedInFrame(), RandomGeometry.nextVector3D(random));
      FrameVector3D originAcceleration = new FrameVector3D(twistOfBodyWithRespectToBase.getExpressedInFrame(), RandomGeometry.nextVector3D(random));
      acceleration.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, twistOfBodyWithRespectToBase);

      FrameVector3D linearAccelerationCheck = new FrameVector3D();
      acceleration.getLinearAccelerationFromOriginAcceleration(twistOfBodyWithRespectToBase, linearAccelerationCheck);
      linearAccelerationCheck.changeFrame(originAcceleration.getReferenceFrame());

      EuclidCoreTestTools.assertTuple3DEquals(linearAccelerationCheck, originAcceleration, 1e-12);
      
      FrameVector3D originAccelerationBack = new FrameVector3D(twistOfBodyWithRespectToBase.getExpressedInFrame());
      FramePoint3D origin = new FramePoint3D(acceleration.getBodyFrame());
      origin.changeFrame(acceleration.getBaseFrame());
      acceleration.changeFrame(acceleration.getBaseFrame(), twistOfBodyWithRespectToBase, twistOfBodyWithRespectToBase);
      twistOfBodyWithRespectToBase.changeFrame(acceleration.getBaseFrame());
      acceleration.getAccelerationOfPointFixedInBodyFrame(twistOfBodyWithRespectToBase, origin, originAccelerationBack);

      originAccelerationBack.changeFrame(originAcceleration.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(originAccelerationBack, originAcceleration, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrameNoRelativeMotion()
   {
      ReferenceFrame bodyFrame = frameA;
      ReferenceFrame baseFrame = frameB;
      ReferenceFrame expressedInFrame = frameC;
      Twist twist = new Twist(bodyFrame, baseFrame, expressedInFrame, RandomGeometry.nextVector3D(random), RandomGeometry.nextVector3D(random));
      SpatialAccelerationVector acceleration = new SpatialAccelerationVector(bodyFrame, baseFrame, expressedInFrame, twist.getLinearPart(),
                                                  twist.getAngularPart());

      twist.changeFrame(frameA);
      acceleration.changeFrameNoRelativeMotion(twist.getExpressedInFrame());

      double epsilon = 1e-12;
      assertEquals(twist.getExpressedInFrame(), acceleration.getExpressedInFrame());
      EuclidCoreTestTools.assertTuple3DEquals(twist.getAngularPart(), acceleration.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(twist.getLinearPart(), acceleration.getLinearPart(), epsilon);
   }

   private static Vector3D numericallyDifferentiate(Vector3D previousLinearVelocity, Vector3D linearVelocity, double deltaT)
   {
      Vector3D ret = new Vector3D(linearVelocity);
      ret.sub(previousLinearVelocity);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private static Vector3D getSinusoidalVelocity(double[] amplitudes, double[] frequencies, double t)
   {
      double[] velocities = new double[3];

      for (int i = 0; i < 3; i++)
      {
         double linearAmplitude = amplitudes[i];
         double linearFrequency = frequencies[i];

         velocities[i] = linearAmplitude * Math.sin(linearFrequency * t);
      }

      return new Vector3D(velocities);
   }

   private static Vector3D getSinusoidalAcceleration(double[] amplitudes, double[] frequencies, double t)
   {
      double[] accelerations = new double[3];

      for (int i = 0; i < 3; i++)
      {
         double linearAmplitude = amplitudes[i];
         double linearFrequency = frequencies[i];

         accelerations[i] = linearFrequency * linearAmplitude * Math.cos(linearFrequency * t);
      }

      return new Vector3D(accelerations);
   }

   private Vector3D getRandomVector(Random random)
   {
      return new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Override
   protected SpatialAccelerationVector createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
           Vector3D linearPart, Vector3D angularPart)
   {
      return new SpatialAccelerationVector(bodyFrame, baseFrame, expressedInFrame, linearPart, angularPart);
   }

   @Override
   protected SpatialAccelerationVector createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
           DenseMatrix64F matrix)
   {
      return new SpatialAccelerationVector(bodyFrame, baseFrame, expressedInFrame, matrix);
   }
}
