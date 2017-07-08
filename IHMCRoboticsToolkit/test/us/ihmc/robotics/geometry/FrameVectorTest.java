package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

public class FrameVectorTest extends FrameTuple3DTest<FrameVector3D, Vector3D>
{
   @Override
   public FrameVector3D createTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return createFrameTuple(referenceFrame, x, y, z);
   }

   @After
   public void tearDown()
   {
   }

   @Override
   public FrameVector3D createEmptyFrameTuple()
   {
      return new FrameVector3D();
   }

   @Override
   public FrameVector3D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FrameVector3D(referenceFrame, x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void allConstructorsWork() //Brett
   {
      Tuple3DBasics tuple = new Point3D(1.0, 1.0, 1.0);
      double[] doubleArray = {10.0, -20.0, 35.0};
      Vector3D zeroVector = new Vector3D(0.0, 0.0, 0.0);

      FrameVector3D frameTuple = new FrameVector3D(theFrame, tuple);
      assertEquals("These should be equal", tuple, frameTuple.getVector());
      assertEquals("These should be equal", theFrame, frameTuple.getReferenceFrame());

      FrameVector3D frameDoubles = new FrameVector3D(theFrame, doubleArray);
      Tuple3DBasics position = new Vector3D(doubleArray);
      assertEquals("These should be equal", position, frameDoubles.getVector());
      assertEquals("These should be equal", theFrame, frameDoubles.getReferenceFrame());

      FrameVector3D frameDoublesString = new FrameVector3D(theFrame, doubleArray);
      assertEquals("These should be equal", position, frameDoublesString.getVector());
      assertEquals("These should be equal", theFrame, frameDoublesString.getReferenceFrame());

      FrameVector3D empty = new FrameVector3D();
      empty.setIncludingFrame(theFrame, 10.0, -20.0, 35.0);
      assertEquals("These should be equal", position, empty.getVector());
      assertEquals("These should be equal", theFrame, empty.getReferenceFrame());

      FrameVector3D frame = new FrameVector3D(theFrame);
      frame.set(10.0, -20.0, 35.0);
      assertEquals("These should be equal", position, frame.getVector());
      assertEquals("These should be equal", theFrame, frame.getReferenceFrame());

      FrameVector3D frameString = new FrameVector3D(theFrame);
      assertEquals("These should be equal", theFrame, frameString.getReferenceFrame());
      assertEquals("These should be equal", zeroVector, frameString.getVector());

      //      FrameVector frametuple = new FrameVector(frameTuple);
      //      FrameVector frameXYZ = new FrameVector(referenceFrame, x, y, z);
      //      FrameVector frameXYZString = new FrameVector(referenceFrame, x, y, z name);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameChanges()
   {
      FrameVector3D frameVector = new FrameVector3D(theFrame);
      
      FrameVector3D result = new FrameVector3D(frameVector);

      result = new FrameVector3D(frameVector);
      result.changeFrame(childFrame);
      result.checkReferenceFrameMatch(childFrame);
      
      frameVector.changeFrame(theFrame); //cause of failure
      frameVector.checkReferenceFrameMatch(theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVector()
   {
      FrameVector3D frameVector = new FrameVector3D(theFrame, 10.0, 20.0, 30.0);
      Vector3D expected = frameVector.getVector();
      assertEquals("These should be equal", 10.0, expected.getX(), epsilon);
      assertEquals("These should be equal", 20.0, expected.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testDot() //Brett
   {
      FrameVector3D frameVector1 = new FrameVector3D(theFrame, 1.0, 2.0, 3.0);
      FrameVector3D frameVector2 = new FrameVector3D(theFrame, 10.0, 20.0, 30.0);
      double dotProduct = frameVector1.dot(frameVector2);
      double expected = 140.0;
      assertEquals("This should be equal", expected, dotProduct, epsilon);
      
      //test for mismatched reference frames
      FrameVector3D frameVector3 = new FrameVector3D(aFrame, 0.0, 1.0, 2.0);
      frameVector1.dot(frameVector3);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAngle() //Brett
   {
      FrameVector3D frameVector1 = new FrameVector3D(theFrame, 1.0, 2.0, 3.0);
      FrameVector3D frameVector2 = new FrameVector3D(theFrame, 0.0, 1.0, 2.0);

      double actual = frameVector1.angle(frameVector2);
      double l1 = frameVector1.length();
      double l2 = frameVector2.length();
      double denom = l1 * l2;
      double numer = frameVector1.dot(frameVector2);
      double arg = numer/denom;
      double expected = Math.acos(arg);
      assertEquals("These should be equal", expected, actual, epsilon);
      
      //test for mismatched reference frames
      FrameVector3D frameVector3 = new FrameVector3D(aFrame, 0.0, 1.0, 2.0);
      frameVector1.angle(frameVector3);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLength() //Brett was here
   {
      for(int i = 0; i < 100; i++) //compare against Vector3d.length()
      {
         Random random = new Random(45456L);
         Vector3D vector3d = RandomGeometry.nextVector3D(random);
         FrameVector3D frameVector= new FrameVector3D(theFrame, vector3d);
         double vector3dResult = vector3d.length();
         double frameVectorResult = frameVector.length();
         assertEquals("These should be equal", vector3dResult, frameVectorResult, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLengthSquared() //Brett was here
   {
      FrameVector3D frameVector = new FrameVector3D(theFrame, 0.0, 4.0, 3.0);
      assertEquals("These should be equal", 25.0, frameVector.lengthSquared(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform() //Brett was here
   {
      for(int i = 0; i < 100; i++)
      {
         Random random = new Random(45456L);
         Vector3D vector3d = RandomGeometry.nextVector3D(random);
         FrameVector3D frameVector = new FrameVector3D(theFrame, vector3d);
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         frameVector.applyTransform(transform);  //Compare transform of Vector3d and FrameVector
         transform.transform(vector3d);
         assertTrue(frameVector.getVector().epsilonEquals(vector3d, epsilon));
      }      
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCrosses() //Brett
   {
      FrameVector3D frameVector1 = createFrameTuple(theFrame, 1, 2, 3); 
      FrameVector3D frameVector2 = createFrameTuple(theFrame, 0, 1, 2);
      FrameVector3D v2other = createFrameTuple(aFrame, 0, 1, 2);
      FrameVector3D result = createFrameTuple(theFrame, 0, 0, 0);
      Vector3D expected = new Vector3D(1, -2, 1);

      result.cross(frameVector1, frameVector2);
      assertTrue(result.getVector().epsilonEquals(expected, epsilon)); //correctly calculates cross product
      assertEquals("These should be equal", 0.0, result.dot(frameVector1), epsilon); //result of cross product orthogonal to both its factors
      assertEquals("These should be equal", 0.0, result.dot(frameVector2), epsilon); //result of cross product orthogonal to both its factors

      try //fails on non-matching reference frames
      {
         result.cross(frameVector1, v2other);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }

      try //fails on non-matching reference frames
      {
         result.cross(v2other, frameVector1);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }

      for(int i = 0; i < 100; i++) //compare against Vector3d.cross()
      {
         Random random = new Random(45456L);
         Vector3D v1 = RandomGeometry.nextVector3D(random);
         Vector3D v2 = RandomGeometry.nextVector3D(random);
         Vector3D v3 = RandomGeometry.nextVector3D(random);
         Vector3D staticResult = RandomGeometry.nextVector3D(random);
         FrameVector3D fv1 = new FrameVector3D(theFrame, v1);
         FrameVector3D fv2 = new FrameVector3D(theFrame, v2);
         FrameVector3D fv3 = new FrameVector3D(theFrame, v3);

         v3.cross(v1, v2);  //Compare cross of Vector3d and FrameVector
         fv3.cross(fv1, fv2);
         assertTrue(fv3.getVector().epsilonEquals(v3, epsilon));

         staticResult.cross(v1, v2); //compare to Vector3D.cross()
         assertTrue(fv3.getVector().epsilonEquals(staticResult, epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCrossWithVector3d()
   {
      Random random = new Random(45456L);
      FrameVector3D expectedFrameVector = new FrameVector3D(theFrame);
      FrameVector3D actualFrameVector = new FrameVector3D(theFrame);

      for(int i = 0; i < 100; i++) //compare against Vector3d.cross()
      {
         Vector3D v1 = RandomGeometry.nextVector3D(random);
         Vector3D v2 = RandomGeometry.nextVector3D(random);
         Vector3D v3 = RandomGeometry.nextVector3D(random);
         

         v3.cross(v1, v2);  //Compare cross of Vector3d and FrameVector
         expectedFrameVector.set(v3);
         actualFrameVector.cross(v1, v2);
         assertTrue(actualFrameVector.epsilonEquals(expectedFrameVector, epsilon));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNormalize() //Brett
   {
      for(int i = 0; i < 100; i++) //sum of squares of normalized vector equals 1
      {
         Random random = new Random(45456L);
         Vector3D vector3d = RandomGeometry.nextVector3D(random);
         FrameVector3D v1 = new FrameVector3D(theFrame, vector3d);
         v1.normalize();
         double sumOfSquares = v1.getX()*v1.getX() + v1.getY()*v1.getY() + v1.getZ()*v1.getZ(); 
         assertEquals("These should be equal", 1.0, sumOfSquares, epsilon);
      }
   }

	@Test
   public void testLimitLength() throws Exception
   {
      Random random = new Random(546456);

      FrameVector3D expectedVector = new FrameVector3D(theFrame);

      for (int i = 0; i < 100; i++)
      {
         double maximumLength = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double smallLength = random.nextDouble() * maximumLength;
         FrameVector3D smallVector = new FrameVector3D(theFrame, EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, smallLength));
         expectedVector.set(smallVector);

         boolean hasBeenLimited = smallVector.clipToMaxLength(maximumLength);

         assertFalse(hasBeenLimited);
         assertTrue(smallVector.length() < maximumLength); // Redundant assertion but this is what the method does at the end.

         assertEquals(expectedVector.getReferenceFrame(), smallVector.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expectedVector.getVector(), smallVector.getVector(), epsilon);

         double bigLength = RandomNumbers.nextDouble(random, 1.0, 10.0) * maximumLength;
         FrameVector3D bigVector = new FrameVector3D(theFrame, EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, bigLength));
         expectedVector.set(bigVector);
         expectedVector.normalize();
         expectedVector.scale(maximumLength);
         
         hasBeenLimited = bigVector.clipToMaxLength(maximumLength);

         assertTrue(hasBeenLimited);
         assertEquals(bigVector.length(), maximumLength, epsilon);

         assertEquals(expectedVector.getReferenceFrame(), bigVector.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expectedVector.getVector(), bigVector.getVector(), epsilon);
      }
   }

   public static void assertFrameVectorEquals(FrameVector3D expected, FrameVector3D actual, double delta)
   {
      expected.checkReferenceFrameMatch(actual);
      EuclidCoreTestTools.assertTuple3DEquals(expected.getVector(), actual.getVector(), delta);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      assertSuperMethodsAreOverloaded(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, FrameVector3D.class, Vector3DBasics.class);
   }
}
