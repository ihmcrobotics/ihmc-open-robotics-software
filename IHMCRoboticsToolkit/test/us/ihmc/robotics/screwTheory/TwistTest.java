package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.After;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.testing.JUnitTools;

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
      Vector3D angularVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist = new Twist(frameB, frameA, frameC, linearVelocity, angularVelocity);

      // test getters
      double epsilon = 1e-14;
      Vector3D angularVelocity2 = twist.getAngularPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, angularVelocity2, epsilon);

      Vector3D linearVelocity2 = twist.getLinearPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, linearVelocity2, epsilon);

      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, angularVelocity2, epsilon);    // make sure the linear velocity setter didn't change the angular velocity

      // test setters
      Vector3D angularVelocity3 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.setAngularPart(angularVelocity3);

      Vector3D linearVelocity3 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.setLinearPart(linearVelocity3);

      Vector3D angularVelocity4 = twist.getAngularPartCopy();
      Vector3D linearVelocity4 = twist.getLinearPartCopy();

      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity3, angularVelocity4, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity3, linearVelocity4, epsilon);
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
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), twist.getAngularPart(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), twist.getLinearPart(), 0.0);
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
      Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
      Twist twist2 = createSpatialMotionVector(frameB, frameA, frameA, new Vector3D(), new Vector3D());

      twist1.add(twist2);
   }

   /**
    * You shouldn't be able to add two twists if the second is not relative to the first
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAddNotRelative()
   {
      Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
      Twist twist2 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());

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
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameD, linearVelocity1, angularVelocity1);

      Vector3D angularVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist2 = new Twist(frameC, frameB, frameD, linearVelocity2, angularVelocity2);

      twist1.add(twist2);

      assertEquals(frameD, twist1.getExpressedInFrame());
      assertEquals(frameA, twist1.getBaseFrame());
      assertEquals(frameC, twist1.getBodyFrame());

      angularVelocity1.add(angularVelocity2);
      linearVelocity1.add(linearVelocity2);

      double epsilon = 1e-14;
      Vector3D angularVelocity1Plus2 = twist1.getAngularPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity1, angularVelocity1Plus2, epsilon);

      Vector3D linearVelocity1Plus2 = twist1.getLinearPartCopy();
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity1, linearVelocity1Plus2, epsilon);

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
         Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
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

         Vector3D omega1InC = new Vector3D();
         Vector3D v1InC = new Vector3D();
         fromTildeForm(twist1TildeInC, omega1InC, v1InC);

         // then transform using 2.8 (c)
         twist1.changeFrame(frameC);
         assertEquals(frameC, twist1.getExpressedInFrame());

         Vector3D omega1InC2 = twist1.getAngularPartCopy();
         Vector3D v1InC2 = twist1.getLinearPartCopy();

         double epsilon = 1e-8;
         EuclidCoreTestTools.assertTuple3DEquals(omega1InC, omega1InC2, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(v1InC, v1InC2, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrameSameFrame()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
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
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
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
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameB, linearVelocity1, angularVelocity1);

      FrameVector expectedFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
      Vector3D expected = expectedFrameVector.getVector();
      twist1.getBodyOriginLinearPartInBaseFrame(expectedFrameVector);

      FrameVector actual = new FrameVector(ReferenceFrame.getWorldFrame());
      twist1.changeFrame(twist1.getBaseFrame());
      FramePoint bodyFrameOrigin = new FramePoint(twist1.getBodyFrame());
      bodyFrameOrigin.changeFrame(twist1.getBaseFrame());
      twist1.getLinearVelocityOfPointFixedInBodyFrame(actual, bodyFrameOrigin);

      EuclidCoreTestTools.assertTuple3DEquals(expected, actual.getVectorCopy(), 1e-6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testBodyOriginLinearPartInBaseFrameAndAngularVelocity()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist = new Twist(frameA, frameB, frameC, linearVelocity1, angularVelocity1);

      Vector3D angularVelocityInBaseFrame = new Vector3D();
      twist.getAngularVelocityInBaseFrame(angularVelocityInBaseFrame);
      FrameVector bodyOriginLinearPart = new FrameVector();
      twist.getBodyOriginLinearPartInBaseFrame(bodyOriginLinearPart);
      assertEquals(twist.getBaseFrame(), bodyOriginLinearPart.getReferenceFrame());

      double dt = 1e-8;
      RigidBodyTransform transform = twist.getBodyFrame().getTransformToDesiredFrame(twist.getBaseFrame());

      RotationMatrix oldRotation = new RotationMatrix();
      RotationMatrix newRotation = new RotationMatrix();
      transform.getRotation(oldRotation);
      transform.getRotation(newRotation);

      Vector3D oldPosition = new Vector3D();
      Vector3D newPosition = new Vector3D();
      transform.getTranslation(oldPosition);
      transform.getTranslation(newPosition);

      ScrewTestTools.integrate(newRotation, newPosition, dt, twist);

      Vector3D velocityNumerical = new Vector3D(newPosition);
      velocityNumerical.sub(oldPosition);
      velocityNumerical.scale(1.0 / dt);
      EuclidCoreTestTools.assertTuple3DEquals(velocityNumerical, bodyOriginLinearPart.getVector(), 1e-5);

      Matrix3D angularVelocityMatrix = new Matrix3D(newRotation);
      angularVelocityMatrix.sub(oldRotation);
      angularVelocityMatrix.scale(1.0 / dt);
      angularVelocityMatrix.multiplyTransposeOther(oldRotation);

      Assert.assertTrue(angularVelocityMatrix.isMatrixSkewSymmetric(1.0e-5));
      Vector3D angularVelocityNumerical = new Vector3D();
      fromTildeForm(angularVelocityNumerical, angularVelocityMatrix);
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocityNumerical, angularVelocityInBaseFrame, 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScrewConstruction()
   {
      double angularVelocityMagnitude = random.nextDouble();
      double linearVelocityMagnitude = 0.0;
      Vector3D axisOfRotation = RandomTools.generateRandomVector(random);
      axisOfRotation.normalize();
      Vector3D offset = RandomTools.generateRandomVector(random);

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
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), velocityOfStationaryPoint.getVector(), delta);
   }

   /**
    * Converts the twist to 'tilde' form, i.e.:
    * [tilde(omega), v;
    *             0, 0];
    */
   private static DenseMatrix64F toTildeForm(Twist twist)
   {
      Vector3D angularVelocity = twist.getAngularPartCopy();
      Vector3D linearVelocity = twist.getLinearPartCopy();

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
   private static void fromTildeForm(DenseMatrix64F twistTilde, Vector3D angularVelocityToPack, Vector3D linearVelocityToPack)
   {
      linearVelocityToPack.set(twistTilde.get(0, 3), twistTilde.get(1, 3), twistTilde.get(2, 3));
      angularVelocityToPack.set(twistTilde.get(2, 1), twistTilde.get(0, 2), twistTilde.get(1, 0));
   }

   private static void fromTildeForm(Vector3D omegaToPack, Matrix3D omegaTilde)
   {
      omegaToPack.set(omegaTilde.getM21(), omegaTilde.getM02(), omegaTilde.getM10());
   }

   @Override
   protected Twist createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3D linearPart,
           Vector3D angularPart)
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
