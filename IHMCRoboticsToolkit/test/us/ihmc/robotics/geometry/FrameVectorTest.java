package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameVectorTest extends FrameTupleTest<Vector3D>
{

   @After
   public void tearDown()
   {
   }

   @Override
   public FrameVector createEmptyFrameTuple()
   {
      return new FrameVector();
   }

   @Override
   public FrameVector createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FrameVector(referenceFrame, x, y, z);
   }

   @Override
   public FrameTuple<FrameVector, Vector3D> createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z, String name)
   {
      return new FrameVector(referenceFrame, x, y, z, name);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void allConstructorsWork() //Brett
   {
      Tuple3DBasics tuple = new Point3D(1.0, 1.0, 1.0);
      String name = "VectorName";
      double[] doubleArray = {10.0, -20.0, 35.0};
      Vector3D zeroVector = new Vector3D(0.0, 0.0, 0.0);

      FrameVector frameTuple = new FrameVector(theFrame, tuple);
      assertEquals("These should be equal", tuple, frameTuple.getVector());
      frameTuple.setName(name);
      assertEquals("These should be equal", name, frameTuple.getName());
      assertEquals("These should be equal", theFrame, frameTuple.getReferenceFrame());

      FrameVector frameTupleString = new FrameVector(theFrame, tuple, name);
      assertEquals("These should be equal", tuple, frameTupleString.getVector());
      frameTupleString.setName(name);
      assertEquals("These should be equal", name, frameTupleString.getName());
      assertEquals("These should be equal", theFrame, frameTupleString.getReferenceFrame());

      FrameVector frameDoubles = new FrameVector(theFrame, doubleArray);
      Tuple3DBasics position = new Vector3D(doubleArray);
      assertEquals("These should be equal", position, frameDoubles.getVector());
      frameDoubles.setName(name);
      assertEquals("These should be equal", name, frameDoubles.getName());
      assertEquals("These should be equal", theFrame, frameDoubles.getReferenceFrame());

      FrameVector frameDoublesString = new FrameVector(theFrame, doubleArray, name);
      assertEquals("These should be equal", position, frameDoublesString.getVector());
      assertEquals("These should be equal", name, frameDoublesString.getName());
      assertEquals("These should be equal", theFrame, frameDoublesString.getReferenceFrame());

      FrameVector empty = new FrameVector();
      empty.setName(name);
      empty.setIncludingFrame(theFrame, 10.0, -20.0, 35.0);
      assertEquals("These should be equal", position, empty.getVector());
      assertEquals("These should be equal", name, empty.getName());
      assertEquals("These should be equal", theFrame, empty.getReferenceFrame());

      FrameVector frame = new FrameVector(theFrame);
      frame.setName(name);
      frame.set(10.0, -20.0, 35.0);
      assertEquals("These should be equal", position, frame.getVector());
      assertEquals("These should be equal", name, frame.getName());
      assertEquals("These should be equal", theFrame, frame.getReferenceFrame());

      FrameVector frameString = new FrameVector(theFrame, name);
      assertEquals("These should be equal", name, frameString.getName());
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
      FrameVector frameVector = new FrameVector(theFrame);
      RigidBodyTransform transform3d = new RigidBodyTransform();
      
      frameVector.changeFrameUsingTransform(childFrame, transform3d);
      frameVector.checkReferenceFrameMatch(childFrame);
      
      FrameVector result = new FrameVector(frameVector);

      result.changeFrameUsingTransform(theFrame, transform3d);
      result.checkReferenceFrameMatch(theFrame);
      
      result = new FrameVector(frameVector);
      result.changeFrame(childFrame);
      result.checkReferenceFrameMatch(childFrame);
      
      frameVector.changeFrame(theFrame); //cause of failure
      frameVector.checkReferenceFrameMatch(theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVector()
   {
      FrameVector frameVector = new FrameVector(theFrame, 10.0, 20.0, 30.0);
      Vector3D expected = frameVector.getVector();
      assertEquals("These should be equal", 10.0, expected.getX(), epsilon);
      assertEquals("These should be equal", 20.0, expected.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToFrameVector2d() //Brett was here
   {
      FrameVector frameVector = new FrameVector(theFrame, 11.0, 22.0, 33.0);
      FrameVector2d expected = frameVector.toFrameVector2d();
      assertEquals("These should be equal", 11.0, expected.getX(), epsilon);
      assertEquals("These should be equal", 22.0, expected.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testDot() //Brett
   {
      FrameVector frameVector1 = new FrameVector(theFrame, 1.0, 2.0, 3.0);
      FrameVector frameVector2 = new FrameVector(theFrame, 10.0, 20.0, 30.0);
      double dotProduct = frameVector1.dot(frameVector2);
      double expected = 140.0;
      assertEquals("This should be equal", expected, dotProduct, epsilon);
      
      //test for mismatched reference frames
      FrameVector frameVector3 = new FrameVector(aFrame, 0.0, 1.0, 2.0);
      frameVector1.dot(frameVector3);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testAngle() //Brett
   {
      FrameVector frameVector1 = new FrameVector(theFrame, 1.0, 2.0, 3.0);
      FrameVector frameVector2 = new FrameVector(theFrame, 0.0, 1.0, 2.0);

      double actual = frameVector1.angle(frameVector2);
      double l1 = frameVector1.length();
      double l2 = frameVector2.length();
      double denom = l1 * l2;
      double numer = frameVector1.dot(frameVector2);
      double arg = numer/denom;
      double expected = Math.acos(arg);
      assertEquals("These should be equal", expected, actual, epsilon);
      
      //test for mismatched reference frames
      FrameVector frameVector3 = new FrameVector(aFrame, 0.0, 1.0, 2.0);
      frameVector1.angle(frameVector3);
   }
	
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsParallel()
   {
      Random random = new Random(100L);

      FrameVector randomFrameVector = new FrameVector(theFrame, RandomGeometry.nextVector3D(random));
      FrameVector parallelVector = new FrameVector(randomFrameVector);
      parallelVector.scale(RandomNumbers.nextDouble(random, -1.0, 1.0));

      String errorMsg = "\n" + randomFrameVector + "\n should be parallel to: \n" + parallelVector + "\n Angle between vectors = " + randomFrameVector.angle(parallelVector);
      assertTrue(errorMsg, randomFrameVector.isEpsilonParallel(parallelVector, 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLength() //Brett was here
   {
      for(int i = 0; i < 100; i++) //compare against Vector3d.length()
      {
         Random random = new Random(45456L);
         Vector3D vector3d = RandomGeometry.nextVector3D(random);
         FrameVector frameVector= new FrameVector(theFrame, vector3d);
         double vector3dResult = vector3d.length();
         double frameVectorResult = frameVector.length();
         assertEquals("These should be equal", vector3dResult, frameVectorResult, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLengthSquared() //Brett was here
   {
      FrameVector frameVector = new FrameVector(theFrame, 0.0, 4.0, 3.0);
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
         FrameVector frameVector = new FrameVector(theFrame, vector3d);
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
      FrameVector frameVector1 = createFrameTuple(theFrame, 1, 2, 3); 
      FrameVector frameVector2 = createFrameTuple(theFrame, 0, 1, 2);
      FrameVector v2other = createFrameTuple(aFrame, 0, 1, 2);
      FrameVector result = createFrameTuple(theFrame, 0, 0, 0);
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
         FrameVector fv1 = new FrameVector(theFrame, v1);
         FrameVector fv2 = new FrameVector(theFrame, v2);
         FrameVector fv3 = new FrameVector(theFrame, v3);

         v3.cross(v1, v2);  //Compare cross of Vector3d and FrameVector
         fv3.cross(fv1, fv2);
         assertTrue(fv3.getVector().epsilonEquals(v3, epsilon));

         FrameVector.cross(staticResult, v1, v2); //compare to static version of cross()
         assertTrue(fv3.getVector().epsilonEquals(staticResult, epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCrossWithVector3d()
   {
      Random random = new Random(45456L);
      FrameVector expectedFrameVector = new FrameVector(theFrame);
      FrameVector actualFrameVector = new FrameVector(theFrame);

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
         FrameVector v1 = new FrameVector(theFrame, vector3d);
         v1.normalize();
         double sumOfSquares = v1.getX()*v1.getX() + v1.getY()*v1.getY() + v1.getZ()*v1.getZ(); 
         assertEquals("These should be equal", 1.0, sumOfSquares, epsilon);
      }
   }

   public static void assertFrameVectorEquals(FrameVector expected, FrameVector actual, double delta)
   {
      expected.checkReferenceFrameMatch(actual);
      EuclidCoreTestTools.assertTuple3DEquals(expected.getVector(), actual.getVector(), delta);
   }
}
