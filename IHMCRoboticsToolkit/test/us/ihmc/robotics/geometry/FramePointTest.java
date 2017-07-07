package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.random.RandomGeometry;

public class FramePointTest extends FrameTuple3DTest<FramePoint, Point3D>
{
   public static double epsilon = 1e-10;

   @Override
   public FramePoint createTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return createFrameTuple(referenceFrame, x, y, z);
   }

   @Override
   public FramePoint createEmptyFrameTuple()
   {
      return new FramePoint();
   }

   @Override
   public FramePoint createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FramePoint(referenceFrame, x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testRunTestMain()
   {
      // create frames
      ReferenceFrame A = ReferenceFrame.constructARootFrame("A");

      RigidBodyTransform transform = new RigidBodyTransform();
      Vector3D euler = new Vector3D(Math.PI / 2.0, 0.0, 0.0);
      transform.setRotationEulerAndZeroTranslation(euler);
      Vector3D translation = new Vector3D(5.0, 0.0, 0.0);
      transform.setTranslation(translation);

      //    System.out.println("B translation = \n" + transform);
      ReferenceFrame B = ReferenceFrame.constructFrameWithUnchangingTransformToParent("B", A, transform);

      transform = new RigidBodyTransform();
      euler = new Vector3D(0.0, Math.PI / 2.0, 0.0);
      transform.setRotationEulerAndZeroTranslation(euler);
      translation = new Vector3D(5.0, 0.0, 0.0);
      transform.setTranslation(translation);

      //    System.out.println("C translation = \n" + transform);
      ReferenceFrame C = ReferenceFrame.constructFrameWithUnchangingTransformToParent("C", B, transform);

      transform = new RigidBodyTransform();
      euler = new Vector3D(0.0, 0.0, Math.PI / 2.0);
      transform.setRotationEulerAndZeroTranslation(euler);
      translation = new Vector3D(5.0, 0.0, 0.0);
      transform.setTranslation(translation);

      //    System.out.println("D translation = \n" + transform);
      ReferenceFrame D = ReferenceFrame.constructFrameWithUnchangingTransformToParent("D", C, transform);

      FramePoint V1 = new FramePoint(D, 2.0, 3.0, 4.0);

      // System.out.println("V1 = " + V1);
      assertEquals(2, V1.getX(), epsilon);
      assertEquals(3, V1.getY(), epsilon);
      assertEquals(4, V1.getZ(), epsilon);

      try
      {
         FramePoint V1inD = new FramePoint(V1);
         V1inD.changeFrame(D);

         //       System.out.println(V1inD);
         assertEquals(2, V1inD.getX(), epsilon);
         assertEquals(3, V1inD.getY(), epsilon);
         assertEquals(4, V1inD.getZ(), epsilon);

         FramePoint V1inC = new FramePoint(V1);
         V1inC.changeFrame(C);

         //       System.out.println(V1inC);
         assertEquals(2, V1inC.getX(), epsilon);
         assertEquals(2, V1inC.getY(), epsilon);
         assertEquals(4, V1inC.getZ(), epsilon);

         FramePoint V1inB = new FramePoint(V1);
         V1inB.changeFrame(B);

         //       System.out.println(V1inB);
         assertEquals(9, V1inB.getX(), epsilon);
         assertEquals(2, V1inB.getY(), epsilon);
         assertEquals(-2, V1inB.getZ(), epsilon);

         FramePoint V1inA = new FramePoint(V1);
         V1inA.changeFrame(A);

         //       System.out.println(V1inA);
         assertEquals(14, V1inA.getX(), epsilon);
         assertEquals(2, V1inA.getY(), epsilon);
         assertEquals(2, V1inA.getZ(), epsilon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         fail();
      }

      V1 = new FramePoint(A, 2.0, 3.0, 4.0);
      assertEquals(2, V1.getX(), epsilon);
      assertEquals(3, V1.getY(), epsilon);
      assertEquals(4, V1.getZ(), epsilon);

      // System.out.println("V2 = " + V1);
      try
      {
         FramePoint V1inA = new FramePoint(V1);
         V1inA.changeFrame(A);
         assertEquals(2, V1inA.getX(), epsilon);
         assertEquals(3, V1inA.getY(), epsilon);
         assertEquals(4, V1inA.getZ(), epsilon);

         FramePoint V1inB = new FramePoint(V1);
         V1inB.changeFrame(B);
         assertEquals(-3, V1inB.getX(), epsilon);
         assertEquals(4, V1inB.getY(), epsilon);
         assertEquals(-3, V1inB.getZ(), epsilon);

         FramePoint V1inC = new FramePoint(V1);
         V1inC.changeFrame(C);
         assertEquals(3, V1inC.getX(), epsilon);
         assertEquals(4, V1inC.getY(), epsilon);
         assertEquals(-8, V1inC.getZ(), epsilon);

         FramePoint V1inD = new FramePoint(V1);
         V1inD.changeFrame(D);
         assertEquals(4, V1inD.getX(), epsilon);
         assertEquals(2, V1inD.getY(), epsilon);
         assertEquals(-8, V1inD.getZ(), epsilon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         fail();
      }

      V1 = new FramePoint(B, 2.0, 3.0, 4.0);
      assertEquals(2, V1.getX(), epsilon);
      assertEquals(3, V1.getY(), epsilon);
      assertEquals(4, V1.getZ(), epsilon);

      try
      {
         FramePoint V1inA = new FramePoint(V1);
         V1inA.changeFrame(A);
         assertEquals(7, V1inA.getX(), epsilon);
         assertEquals(-4, V1inA.getY(), epsilon);
         assertEquals(3, V1inA.getZ(), epsilon);
         FramePoint V1inB = new FramePoint(V1);
         V1inB.changeFrame(B);
         assertEquals(2, V1inB.getX(), epsilon);
         assertEquals(3, V1inB.getY(), epsilon);
         assertEquals(4, V1inB.getZ(), epsilon);
         FramePoint V1inC = new FramePoint(V1);
         V1inC.changeFrame(C);
         assertEquals(-4, V1inC.getX(), epsilon);
         assertEquals(3, V1inC.getY(), epsilon);
         assertEquals(-3, V1inC.getZ(), epsilon);
         FramePoint V1inD = new FramePoint(V1);
         V1inD.changeFrame(D);
         assertEquals(3, V1inD.getX(), epsilon);
         assertEquals(9, V1inD.getY(), epsilon);
         assertEquals(-3, V1inD.getZ(), epsilon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         fail();
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOtherConstructors() //Brett was here
   {
      Tuple3DBasics position = new Point3D(1.0, 1.0, 1.0);
      FramePoint framePosition = new FramePoint(theFrame, position);
      assertEquals("These should be equal", position, framePosition.getPoint());
      assertEquals("These should be equal", theFrame, framePosition.getReferenceFrame());

      FramePoint framePositionName = new FramePoint(aFrame, position);
      assertEquals("These should be equal", position, framePositionName.getPoint());
      assertEquals("These should be equal", aFrame, framePositionName.getReferenceFrame());

      double[] doubleArray = { 7.0, 7.0, 7.0 };
      Tuple3DBasics position2 = new Point3D(doubleArray);
      FramePoint framePositionArray = new FramePoint(theFrame, doubleArray);
      assertEquals("These should be equal", position2, framePositionArray.getPoint());
      assertEquals("These should be equal", theFrame, framePositionArray.getReferenceFrame());

      double[] doubleArray2 = { -7.0, 14.0, 21.0 };
      Tuple3DBasics position3 = new Point3D(doubleArray2);
      FramePoint framePositionArrayName = new FramePoint(theFrame, doubleArray2);
      assertEquals("These should be equal", position3, framePositionArrayName.getPoint());
      assertEquals("These should be equal", theFrame, framePositionArrayName.getReferenceFrame());

      FramePoint frameName = new FramePoint(theFrame);
      Tuple3DBasics position4 = new Point3D();
      assertEquals("These should be equal", position4, frameName.getPoint());
      assertEquals("These should be equal", theFrame, frameName.getReferenceFrame());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public void testGetXYplaneDistance()
   {
      FramePoint firstPoint = new FramePoint(theFrame, 1.0, 2.0, 5.0);
      FramePoint secondPoint = new FramePoint(theFrame, 4.0, -2.0, -3.0);

      assertEquals(5.0, firstPoint.distanceXY(secondPoint), epsilon);

      //Test for reference frame mismatch
      FramePoint thirdPoint = new FramePoint(aFrame, 4.0, -2.0, -3.0);
      firstPoint.distanceXY(thirdPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public void testDistance() //Brett
   {
      FramePoint framePoint1 = new FramePoint(theFrame);
      FramePoint framePoint2 = new FramePoint(aFrame);

      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 3.0);
      double expectedReturn = Math.sqrt(14.0);
      double actualReturn = framePoint1.distance(framePoint);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      //Test for reference frame mismatch
      framePoint1.distance(framePoint2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public void testDistanceSquared() //Brett
   {
      FramePoint framePoint1 = new FramePoint(theFrame);
      FramePoint framePoint2 = new FramePoint(aFrame);

      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 3.0);
      double expectedReturn = 14.0;
      double actualReturn = framePoint.distanceSquared(framePoint1);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      //Test for reference frame mismatch
      framePoint1.distanceSquared(framePoint2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testToFramePoint2d() //Brett
   {
      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 5.0);
      FramePoint2d framePoint2d = framePoint.toFramePoint2d();

      assertSame(theFrame, framePoint2d.getReferenceFrame());
      assertEquals(1.0, framePoint2d.getX(), epsilon);
      assertEquals(2.0, framePoint2d.getY(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPoint()
   {
      Point3D tuple3d = new Point3D(1.0, 1.0, 1.0);
      Point3D tuple3dCopy = new Point3D();
      FramePoint framePoint = new FramePoint(theFrame, tuple3d);

      tuple3dCopy = framePoint.getPoint();
      assertTrue(tuple3d.epsilonEquals(tuple3dCopy, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFrameChanges()
   {
      FramePoint framePoint = new FramePoint(theFrame);
      FramePoint result = new FramePoint(framePoint);

      result = new FramePoint(framePoint);
      result.changeFrame(theFrame);
      result.checkReferenceFrameMatch(theFrame);

      framePoint.changeFrame(theFrame); //cause of failure
      framePoint.checkReferenceFrameMatch(theFrame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testApplyTransform()
   {
      FramePoint frameTuple = new FramePoint(theFrame);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3D(2.0, 4.0, 8.0));
      frameTuple.applyTransform(transform);

      assertEquals(2.0, frameTuple.getX(), epsilon);
      assertEquals(4.0, frameTuple.getY(), epsilon);
      assertEquals(8.0, frameTuple.getZ(), epsilon);
      frameTuple.applyTransform(transform);
      assertEquals(4.0, frameTuple.getX(), epsilon);
      assertEquals(8.0, frameTuple.getY(), epsilon);
      assertEquals(16.0, frameTuple.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testApplyTransformScale()
   {
      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 3.0);
      AffineTransform transform3D = new AffineTransform();
      transform3D.setScale(0.1, 0.01, 0.001);

      FramePoint resultPoint = new FramePoint(framePoint);
      resultPoint.applyTransform(transform3D);
      FramePoint expectedResultPoint = new FramePoint(theFrame, 0.1, 0.02, 0.003);

      assertNotSame(framePoint, resultPoint);
      assertTrue(expectedResultPoint.epsilonEquals(resultPoint, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testApplyTransformTranslate()
   {
      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 3.0);
      AffineTransform transform3D = new AffineTransform();

      Vector3D translateVector = new Vector3D(0.1, 0.5, 0.9);
      transform3D.setTranslation(translateVector);
      FramePoint resultPoint = new FramePoint(framePoint);
      resultPoint.applyTransform(transform3D);
      FramePoint expectedResultPoint = new FramePoint(theFrame, 1.1, 2.5, 3.9);

      assertNotSame(framePoint, resultPoint);
      assertTrue(expectedResultPoint.epsilonEquals(resultPoint, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testApplyTransformRotateZ()
   {
      FramePoint framePoint = new FramePoint(theFrame, 1.0, 2.0, 3.0);
      RigidBodyTransform transform3D = new RigidBodyTransform();

      Vector3D rotateVector = new Vector3D(0.0, 0.0, Math.PI / 2.0);
      transform3D.setRotationEulerAndZeroTranslation(rotateVector);
      FramePoint resultPoint = new FramePoint(framePoint);
      resultPoint.applyTransform(transform3D);
      FramePoint expectedResultPoint = new FramePoint(theFrame, -2.0, 1.0, 3.0);

      assertNotSame(framePoint, resultPoint);
      assertTrue(expectedResultPoint.epsilonEquals(resultPoint, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      Random random = new Random();
      double[] xyz = RandomNumbers.nextDoubleArray(random, 3, Double.MAX_VALUE);
      FramePoint pointToBeTested;
      ReferenceFrame referenceFrame = null;
      pointToBeTested = new FramePoint(referenceFrame, xyz);

      pointToBeTested = new FramePoint(aFrame, xyz);
      Point3D point3dExpected = new Point3D(xyz);
      assertTrue(aFrame == pointToBeTested.getReferenceFrame());
      assertTrue(pointToBeTested.getPoint().epsilonEquals(point3dExpected, epsilon));

      double max = Double.MAX_VALUE / 2.0;
      point3dExpected = RandomGeometry.nextPoint3D(random, max, max, max);
      pointToBeTested = new FramePoint(referenceFrame, point3dExpected);

      pointToBeTested = new FramePoint(aFrame, point3dExpected);
      assertTrue(aFrame == pointToBeTested.getReferenceFrame());
      assertTrue("Expected: " + point3dExpected + ", actual: " + pointToBeTested.getPoint(), point3dExpected.epsilonEquals(pointToBeTested.getPoint(), epsilon));

      xyz = RandomNumbers.nextDoubleArray(random, 3, Double.MAX_VALUE);
      pointToBeTested = new FramePoint(referenceFrame, xyz);

      pointToBeTested = new FramePoint(aFrame, xyz);
      point3dExpected = new Point3D(xyz);
      assertTrue(aFrame == pointToBeTested.getReferenceFrame());
      assertTrue(pointToBeTested.getPoint().epsilonEquals(point3dExpected, epsilon));
   }

   public static void assertFramePointEquals(FramePoint expected, FramePoint actual, double delta)
   {
      expected.checkReferenceFrameMatch(actual);
      EuclidCoreTestTools.assertTuple3DEquals(expected.getPoint(), actual.getPoint(), delta);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      assertSuperMethodsAreOverloaded(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, FramePoint.class, Point3DBasics.class);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(FramePoint.class, FramePointTest.class);
   }
}
