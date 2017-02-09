package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.fail;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class TwistTest extends SpatialMotionVectorTest
{
   @After
   public void tearDown() throws Exception
   {
   }

   /**
    * Basic test of constructor, getters and setters
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructionAndGettersAndSetters()
   {
      // construct a random twist that expresses the motion of frame A with respect to B, expressed in frame C
      Vector3d angularVelocity = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist = new Twist(frameB, frameA, frameC, linearVelocity, angularVelocity);

      // test getters
      double epsilon = 1e-14;
      Vector3d angularVelocity2 = twist.getAngularPartCopy();
      JUnitTools.assertTuple3dEquals(angularVelocity, angularVelocity2, epsilon);

      Vector3d linearVelocity2 = twist.getLinearPartCopy();
      JUnitTools.assertTuple3dEquals(linearVelocity, linearVelocity2, epsilon);

      JUnitTools.assertTuple3dEquals(angularVelocity, angularVelocity2, epsilon);    // make sure the linear velocity setter didn't change the angular velocity

      // test setters
      Vector3d angularVelocity3 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.setAngularPart(angularVelocity3);

      Vector3d linearVelocity3 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.setLinearPart(linearVelocity3);

      Vector3d angularVelocity4 = twist.getAngularPartCopy();
      Vector3d linearVelocity4 = twist.getLinearPartCopy();

      JUnitTools.assertTuple3dEquals(angularVelocity3, angularVelocity4, epsilon);
      JUnitTools.assertTuple3dEquals(linearVelocity3, linearVelocity4, epsilon);
   }

   /**
    * default constructor
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDefaultConstructor()
   {
      Twist twist = new Twist();
      assertNull(twist.getBaseFrame());
      assertNull(twist.getBodyFrame());
      assertNull(twist.getExpressedInFrame());
      JUnitTools.assertTuple3dEquals(new Vector3d(), twist.getAngularPart(), 0.0);
      JUnitTools.assertTuple3dEquals(new Vector3d(), twist.getLinearPart(), 0.0);
   }

   /**
    * Constructing using a double array
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructUsingArray()
   {
      double[] array = new double[Twist.SIZE];
      for (int i = 0; i < array.length; i++)
      {
         array[i] = random.nextDouble();
      }

      Twist twist = new Twist(frameC, frameD, frameA, array);
      DenseMatrix64F matrixBack = new DenseMatrix64F(Twist.SIZE, 1);
      twist.getMatrix(matrixBack, 0);
      double[] arrayBack = matrixBack.getData();
      assertArrayEquals(array, arrayBack, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testConstructUsingArrayTooSmall()
   {
      double[] array = new double[Twist.SIZE - 1];
      new Twist(frameC, frameD, frameA, array);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testConstructUsingArrayTooBig()
   {
      double[] array = new double[Twist.SIZE + 1];
      new Twist(frameC, frameD, frameA, array);
   }

   /**
    * Copy constructor
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyConstructor()
   {
      DenseMatrix64F inputMatrix = RandomMatrices.createRandom(Twist.SIZE, 1, random);
      Twist twist = new Twist(frameC, frameD, frameA, inputMatrix);
      Twist twistCopy = new Twist(twist);

      DenseMatrix64F twistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist.getMatrix(twistMatrix, 0);

      DenseMatrix64F twistCopyMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      twistCopy.getMatrix(twistCopyMatrix, 0);

      // test that they're the same
      JUnitTools.assertMatrixEquals(twistMatrix, twistCopyMatrix, 0.0);
      assertEquals(twist.getBodyFrame(), twistCopy.getBodyFrame());
      assertEquals(twist.getExpressedInFrame(), twistCopy.getExpressedInFrame());
      assertEquals(twist.getBaseFrame(), twistCopy.getBaseFrame());

      // test that we're actually copying, not just using references
      inputMatrix = RandomMatrices.createRandom(Twist.SIZE, 1, random);
      twist.set(frameD, frameA, frameC, inputMatrix, 0);
      twist.getMatrix(twistMatrix, 0);
      twistCopy.getMatrix(twistCopyMatrix, 0);

      for (int i = 0; i < twistMatrix.getNumElements(); i++)
      {
         if (twistMatrix.get(i) == twistCopyMatrix.get(i))
            fail();
      }

      assertNotSame(twist.getBodyFrame(), twistCopy.getBodyFrame());
      assertNotSame(twist.getExpressedInFrame(), twistCopy.getExpressedInFrame());
      assertNotSame(twist.getBaseFrame(), twistCopy.getBaseFrame());
   }

   /**
    * Dot product
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDotProduct()
   {
      WrenchTest.testDotProduct(frameA, frameB, frameC);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testDotProductNotAllowed1()
   {
      WrenchTest.testDotProductNotAllowed1(frameA, frameB, frameC);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testDotProductNotAllowed2()
   {
      WrenchTest.testDotProductNotAllowed2(frameA, frameB, frameC);
   }


   /**
    * You shouldn't be able to add two twists expressed in different frames
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAddExpressedInDifferentFrames()
   {
      Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3d(), new Vector3d());
      Twist twist2 = createSpatialMotionVector(frameB, frameA, frameA, new Vector3d(), new Vector3d());

      twist1.add(twist2);
   }

   /**
    * You shouldn't be able to add two twists if the second is not relative to the first
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAddNotRelative()
   {
      Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3d(), new Vector3d());
      Twist twist2 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3d(), new Vector3d());

      twist1.add(twist2);
   }

   /**
    * Test adding two twists, both expressed in the same reference frame, and the second relative to the first
    * (which is allowed)
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAdd()
   {
      Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameD, linearVelocity1, angularVelocity1);

      Vector3d angularVelocity2 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity2 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist2 = new Twist(frameC, frameB, frameD, linearVelocity2, angularVelocity2);

      twist1.add(twist2);

      assertEquals(frameD, twist1.getExpressedInFrame());
      assertEquals(frameA, twist1.getBaseFrame());
      assertEquals(frameC, twist1.getBodyFrame());

      angularVelocity1.add(angularVelocity2);
      linearVelocity1.add(linearVelocity2);

      double epsilon = 1e-14;
      Vector3d angularVelocity1Plus2 = twist1.getAngularPartCopy();
      JUnitTools.assertTuple3dEquals(angularVelocity1, angularVelocity1Plus2, epsilon);

      Vector3d linearVelocity1Plus2 = twist1.getLinearPartCopy();
      JUnitTools.assertTuple3dEquals(linearVelocity1, linearVelocity1Plus2, epsilon);

      // Should throw exception if try it the other way:

      twist1 = new Twist(frameB, frameA, frameD, linearVelocity1, angularVelocity1);
      twist2 = new Twist(frameC, frameB, frameD, linearVelocity2, angularVelocity2);

      try
      {
         twist2.add(twist1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }

      twist1 = new Twist(frameB, frameA, frameD, linearVelocity1, angularVelocity1);

      try
      {
         twist1.add(twist1);

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
      Twist twist1 = new Twist(frameB, frameA, frameD, RandomTools.generateRandomVector(random), RandomTools.generateRandomVector(random));
      Twist twist2 = new Twist(frameC, frameB, frameD, RandomTools.generateRandomVector(random), RandomTools.generateRandomVector(random));
      Twist twist3 = new Twist(twist1);
      twist3.add(twist2);

      double epsilon = 1e-15;

      Twist twist2Back = new Twist(twist3);
      twist2Back.sub(twist1);
      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(twist2, twist2Back, epsilon);

      Twist twist1Back = new Twist(twist3);
      twist1Back.sub(twist2);
      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(twist1, twist1Back, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSubWrongExpressedInFrame()
   {
      Twist twist1 = new Twist(frameB, frameA, frameD);
      Twist twist2 = new Twist(frameB, frameC, frameC);
      twist1.sub(twist2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSubFramesDontMatchUp()
   {
      Twist twist1 = new Twist(frameD, frameA, frameC);
      Twist twist2 = new Twist(frameB, frameC, frameC);
      twist1.sub(twist2);
   }

   /**
    * Test changing frames by comparing the results of changeExpressedInWhatReferenceFrame()
    * with the results of the 'tilde' formula from
    * Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 25, lemma 2.8 (b)
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrame()
   {
      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Twist twist1 = new Twist(frameB, frameA, frameA, linearVelocity1, angularVelocity1);

         // first transform using 2.8 (b)
         DenseMatrix64F twist1TildeInA = toTildeForm(twist1);
         RigidBodyTransform transformFromAToC = frameA.getTransformToDesiredFrame(frameC);
         DenseMatrix64F transformFromAToCMatrix = new DenseMatrix64F(4, 4);
         transformFromAToC.get(transformFromAToCMatrix);
         DenseMatrix64F transformFromAToCMatrixInv = new DenseMatrix64F(transformFromAToCMatrix);
         CommonOps.invert(transformFromAToCMatrixInv);

         DenseMatrix64F twist1TildeInC = new DenseMatrix64F(transformFromAToCMatrix);
         DenseMatrix64F temp = new DenseMatrix64F(transformFromAToCMatrix);
         CommonOps.mult(twist1TildeInC, twist1TildeInA, temp);
         CommonOps.mult(temp, transformFromAToCMatrixInv, twist1TildeInC);

         Vector3d omega1InC = new Vector3d();
         Vector3d v1InC = new Vector3d();
         fromTildeForm(twist1TildeInC, omega1InC, v1InC);

         // then transform using 2.8 (c)
         twist1.changeFrame(frameC);
         assertEquals(frameC, twist1.getExpressedInFrame());

         Vector3d omega1InC2 = twist1.getAngularPartCopy();
         Vector3d v1InC2 = twist1.getLinearPartCopy();

         double epsilon = 1e-8;
         JUnitTools.assertTuple3dEquals(omega1InC, omega1InC2, epsilon);
         JUnitTools.assertTuple3dEquals(v1InC, v1InC2, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrameSameFrame()
   {
      Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameA, linearVelocity1, angularVelocity1);
      Twist twist2 = new Twist(twist1);
      twist1.changeFrame(twist1.getExpressedInFrame());

      DenseMatrix64F twist1Matrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist1.getMatrix(twist1Matrix, 0);

      DenseMatrix64F twist2Matrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist2.getMatrix(twist2Matrix, 0);

      // test that they're the same
      JUnitTools.assertMatrixEquals(twist1Matrix, twist2Matrix, 0.0);
      assertEquals(twist1.getBodyFrame(), twist2.getBodyFrame());
      assertEquals(twist1.getExpressedInFrame(), twist2.getExpressedInFrame());
      assertEquals(twist1.getBaseFrame(), twist2.getBaseFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetMatrix()
   {
      Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameA, linearVelocity1, angularVelocity1);

      DenseMatrix64F twistMatrix = twist1.toMatrix();

      double epsilon = 1e-14;

      assertEquals(angularVelocity1.getX(), twistMatrix.get(0, 0), epsilon);
      assertEquals(angularVelocity1.getY(), twistMatrix.get(1, 0), epsilon);
      assertEquals(angularVelocity1.getZ(), twistMatrix.get(2, 0), epsilon);
      assertEquals(linearVelocity1.getX(), twistMatrix.get(3, 0), epsilon);
      assertEquals(linearVelocity1.getY(), twistMatrix.get(4, 0), epsilon);
      assertEquals(linearVelocity1.getZ(), twistMatrix.get(5, 0), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVelocityOfPointConsistency()
   {
      Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameB, linearVelocity1, angularVelocity1);

      FrameVector expectedFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
      Vector3d expected = expectedFrameVector.getVector();
      twist1.getBodyOriginLinearPartInBaseFrame(expectedFrameVector);

      FrameVector actual = new FrameVector(ReferenceFrame.getWorldFrame());
      twist1.changeFrame(twist1.getBaseFrame());
      FramePoint bodyFrameOrigin = new FramePoint(twist1.getBodyFrame());
      bodyFrameOrigin.changeFrame(twist1.getBaseFrame());
      twist1.getLinearVelocityOfPointFixedInBodyFrame(actual, bodyFrameOrigin);

      JUnitTools.assertTuple3dEquals(expected, actual.getVectorCopy(), 1e-6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testBodyOriginLinearPartInBaseFrameAndAngularVelocity()
   {
      Vector3d angularVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d linearVelocity1 = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist = new Twist(frameA, frameB, frameC, linearVelocity1, angularVelocity1);

      Vector3d angularVelocityInBaseFrame = new Vector3d();
      twist.getAngularVelocityInBaseFrame(angularVelocityInBaseFrame);
      FrameVector bodyOriginLinearPart = new FrameVector();
      twist.getBodyOriginLinearPartInBaseFrame(bodyOriginLinearPart);
      assertEquals(twist.getBaseFrame(), bodyOriginLinearPart.getReferenceFrame());

      double dt = 1e-8;
      RigidBodyTransform transform = twist.getBodyFrame().getTransformToDesiredFrame(twist.getBaseFrame());

      Matrix3d oldRotation = new Matrix3d();
      Matrix3d newRotation = new Matrix3d();
      transform.getRotation(oldRotation);
      transform.getRotation(newRotation);

      Vector3d oldPosition = new Vector3d();
      Vector3d newPosition = new Vector3d();
      transform.getTranslation(oldPosition);
      transform.getTranslation(newPosition);

      ScrewTestTools.integrate(newRotation, newPosition, dt, twist);

      Vector3d velocityNumerical = new Vector3d(newPosition);
      velocityNumerical.sub(oldPosition);
      velocityNumerical.scale(1.0 / dt);
      JUnitTools.assertTuple3dEquals(velocityNumerical, bodyOriginLinearPart.getVector(), 1e-5);

      Matrix3d angularVelocityMatrix = new Matrix3d(newRotation);
      angularVelocityMatrix.sub(oldRotation);
      angularVelocityMatrix.mul(1.0 / dt);
      angularVelocityMatrix.mulTransposeRight(angularVelocityMatrix, oldRotation);

      JUnitTools.assertSkewSymmetric(angularVelocityMatrix, 1e-5);
      Vector3d angularVelocityNumerical = new Vector3d();
      fromTildeForm(angularVelocityNumerical, angularVelocityMatrix);
      JUnitTools.assertTuple3dEquals(angularVelocityNumerical, angularVelocityInBaseFrame, 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScrewConstruction()
   {
      double angularVelocityMagnitude = random.nextDouble();
      double linearVelocityMagnitude = 0.0;
      Vector3d axisOfRotation = RandomTools.generateRandomVector(random);
      axisOfRotation.normalize();
      Vector3d offset = RandomTools.generateRandomVector(random);

      Twist twist = new Twist(frameB, frameA, frameB, angularVelocityMagnitude, linearVelocityMagnitude, axisOfRotation, offset);
      twist.changeFrame(frameA);

      FrameVector offsetAlongAxis = new FrameVector(frameB, axisOfRotation);
      offsetAlongAxis.scale(random.nextDouble());
      FramePoint pointThatShouldBeStationary = new FramePoint(frameB, offset);
      pointThatShouldBeStationary.add(offsetAlongAxis);
      pointThatShouldBeStationary.changeFrame(frameA);

      FrameVector velocityOfStationaryPoint = new FrameVector(frameB);
      twist.getLinearVelocityOfPointFixedInBodyFrame(velocityOfStationaryPoint, pointThatShouldBeStationary);

      double delta = 1e-15;
      JUnitTools.assertTuple3dEquals(new Vector3d(), velocityOfStationaryPoint.getVector(), delta);
   }

   /**
    * Converts the twist to 'tilde' form, i.e.:
    * [tilde(omega), v;
    *             0, 0];
    */
   private static DenseMatrix64F toTildeForm(Twist twist)
   {
      Vector3d angularVelocity = twist.getAngularPartCopy();
      Vector3d linearVelocity = twist.getLinearPartCopy();

      DenseMatrix64F ret = new DenseMatrix64F(4, 4);
      MatrixTools.vectorToSkewSymmetricMatrix(ret, angularVelocity);
      ret.set(0, 3, linearVelocity.getX());
      ret.set(1, 3, linearVelocity.getY());
      ret.set(2, 3, linearVelocity.getZ());
      return ret;
   }

   /**
    * Converts a twist in tilde form back to twist coordinates (angular velocity and linear velocity)
    */
   private static void fromTildeForm(DenseMatrix64F twistTilde, Vector3d angularVelocityToPack, Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(twistTilde.get(0, 3), twistTilde.get(1, 3), twistTilde.get(2, 3));
      angularVelocityToPack.set(twistTilde.get(2, 1), twistTilde.get(0, 2), twistTilde.get(1, 0));
   }

   private static void fromTildeForm(Vector3d omegaToPack, Matrix3d omegaTilde)
   {
      omegaToPack.set(omegaTilde.getM21(), omegaTilde.getM02(), omegaTilde.getM10());
   }

   @Override
   protected Twist createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3d linearPart,
           Vector3d angularPart)
   {
      return new Twist(bodyFrame, baseFrame, expressedInFrame, linearPart, angularPart);
   }

   @Override
   protected SpatialMotionVector createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
           DenseMatrix64F matrix)
   {
      return new GenericSpatialMotionVector(bodyFrame, baseFrame, expressedInFrame, matrix);
   }
}
