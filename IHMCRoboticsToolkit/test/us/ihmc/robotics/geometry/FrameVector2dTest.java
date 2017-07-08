package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnlyTest;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;

public class FrameVector2dTest extends FrameTuple2DTest<FrameVector2D>
{
   @Override
   public FrameVector2D createTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return createFrameTuple(referenceFrame, x, y);
   }

   @Override
   public FrameVector2D createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new FrameVector2D(referenceFrame, x, y);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }

   @After
   public void tearDown()
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_ReferenceFrame_double_double_String()
   {
      double x = 5.7, y = 56.3;
      FrameVector2D frame = new FrameVector2D(theFrame, x, y);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_ReferenceFrame_double_double()
   {
      double x = 5.7, y = 56.3;
      FrameVector2D frame = new FrameVector2D(theFrame, x, y);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d()
   {
      FrameVector2D frame = new FrameVector2D();
      assertEquals("Should be equal", frame.getX(), 0.0, epsilon);
      assertEquals("Should be equal", frame.getY(), 0.0, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), ReferenceFrame.getWorldFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_ReferenceFrame_Tuple2d()
   {
      double x = 5.7, y = 56.3;
      Point2D tuple = new Point2D(x, y);
      FrameVector2D frame = new FrameVector2D(theFrame, tuple);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_ReferenceFrame_double()
   {
      double x = 5.7, y = 56.3;
      double[] vector = {x, y};
      FrameVector2D frame = new FrameVector2D(theFrame, vector);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_ReferenceFrame()
   {
      double x = 0.0, y = 0.0;
      FrameVector2D frame = new FrameVector2D(theFrame);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), theFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFrameVector2d_FrameTuple2d()
   {
      double x = 5.7, y = 56.3;
      FrameTuple2D<?, ?> frameTuple = createFrameTuple(theFrame, x, y);
      FrameVector2D frame = new FrameVector2D(frameTuple);
      assertEquals("Should be equal", frame.getX(), x, epsilon);
      assertEquals("Should be equal", frame.getY(), y, epsilon);
      assertEquals("Should be equal", frame.getReferenceFrame(), frameTuple.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVector()
   {
      double x = 5.6, y = 45.67;
      FrameVector2D frame = new FrameVector2D(aFrame, x, y);
      Vector2DReadOnly vector2d = frame.getVector();
      assertEquals("Should be equal", frame.getX(), vector2d.getX(), epsilon);
      assertEquals("Should be equal", frame.getY(), vector2d.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDot_FrameVector2d()
   {
      Random random = new Random(4556L);
      double x1 = random.nextDouble(), x2 = random.nextDouble(), x3 = random.nextDouble(), 
            y1 = random.nextDouble(), y2 = random.nextDouble(), y3 = random.nextDouble();
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);
      FrameVector2D frame2 = new FrameVector2D(theFrame, x2, y2);
      FrameVector2D frame3 = new FrameVector2D(aFrame, x3, y3);

      double result = frame1.dot(frame2);
      assertEquals("Should be equal", result, frame1.getX() * frame2.getX() + frame1.getY() * frame2.getY(), epsilon);
      try
      {
         frame1.dot(frame3);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCross_FrameVector2d()
   {
      Random random = new Random(4556L);
      double x1 = random.nextDouble(), x2 = random.nextDouble(), x3 = random.nextDouble(), 
            y1 = random.nextDouble(), y2 = random.nextDouble(), y3 = random.nextDouble();
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);
      FrameVector2D frame2 = new FrameVector2D(theFrame, x2, y2);
      FrameVector2D frame3 = new FrameVector2D(aFrame, x3, y3);

      double result = frame1.cross(frame2);
      assertEquals("Should be equal", result, frame1.getX() * frame2.getY() - frame1.getY() * frame2.getX(), epsilon);
      try
      {
         frame1.cross(frame3);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAngle_FrameVector2d()
   {
      Random random = new Random(4556L);
      double x1 = random.nextDouble(), x2 = random.nextDouble(), x3 = random.nextDouble(), 
            y1 = random.nextDouble(), y2 = random.nextDouble(), y3 = random.nextDouble();
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);
      FrameVector2D frame2 = new FrameVector2D(theFrame, x2, y2);
      FrameVector2D frame3 = new FrameVector2D(aFrame, x3, y3);
      
      Vector2D vector1 = new Vector2D(x1, y1);
      Vector2D vector2 = new Vector2D(x2, y2);

      double result = frame1.angle(frame2);
      double resultVector = vector1.angle(vector2);
      assertEquals("Should be equal", result, resultVector, epsilon);
      try
      {
         frame1.angle(frame3);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }

      for (int i = 0; i<1000; i++)
      {
         double firstVectorLength = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double secondVectorLength = RandomNumbers.nextDouble(random, 0.0, 10.0);
         FrameVector2D firstVector = EuclidFrameRandomTools.generateRandomFrameVector2D(random, worldFrame);
         firstVector.scale(firstVectorLength / firstVector.length());
         FrameVector2D secondVector = new FrameVector2D();

         for (double yaw = -Math.PI; yaw <= Math.PI; yaw += Math.PI / 100.0)
         {
            double c = Math.cos(yaw);
            double s = Math.sin(yaw);
            secondVector.setX(firstVector.getX() * c - firstVector.getY() * s);
            secondVector.setY(firstVector.getX() * s + firstVector.getY() * c);
            secondVector.scale(secondVectorLength / firstVectorLength);
            double computedYaw = firstVector.angle(secondVector);
            double yawDifference = AngleTools.computeAngleDifferenceMinusPiToPi(yaw, computedYaw);
            assertEquals(0.0, yawDifference, 1.0e-12);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNormalize()
   {
      double x1 = 1.0, y1 = 1.0;
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);

      frame1.normalize();
      double result = frame1.getX() * frame1.getX() + frame1.getY() * frame1.getY();
      assertEquals("Should be equal", 1.0, result, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLength()
   {
      double x1 = 1.0, y1 = 1.0;
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);
      double result = Math.sqrt(frame1.getX() * frame1.getX() + frame1.getY() * frame1.getY());
      assertEquals("Should be equal", result, frame1.length(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLengthSquared()
   {
      double x1 = 1.0, y1 = 1.0;
      FrameVector2D frame1 = new FrameVector2D(theFrame, x1, y1);
      double result = frame1.getX() * frame1.getX() + frame1.getY() * frame1.getY();
      assertEquals("Should be equal", result, frame1.lengthSquared(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrame_ReferenceFrame()
   {
      FrameVector2D frameVector = new FrameVector2D(theFrame);
      frameVector.changeFrame(childFrame);
      frameVector.checkReferenceFrameMatch(childFrame);

      frameVector.changeFrame(childFrame);
      frameVector.checkReferenceFrameMatch(childFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform_Transform3D()
   {  
      Random random = new Random(398742498237598750L);
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      Vector3D vectorToTransform = RandomGeometry.nextVector3D(random, 0.0, 0.0, 0.0, 100.0, 100.0, 0.0);
      FrameVector2D vectorToTest = new FrameVector2D(null, new Vector2D(vectorToTransform.getX(), vectorToTransform.getY())); 

      try
      {
         vectorToTest.applyTransform(transform);
         fail("Should have thrown RuntimeException");
      }
      catch(RuntimeException re)
      {
         //Good
      }

      double[] matrix = {6.0, 7.0, 0.0};
      RigidBodyTransform transform2 = EuclidCoreRandomTools.generateRandomRigidBodyTransform2D(random);

      Vector3D vectorToTransform2 = new Vector3D(matrix);
      FrameVector2D vectorToTest2 = new FrameVector2D(null, matrix);

      transform2.transform(vectorToTransform2);
      vectorToTest2.applyTransform(transform2);

      assertEquals("Should be equal", vectorToTransform2.getX(), vectorToTest2.getX(), epsilon);
      assertEquals("Should be equal", vectorToTransform2.getY(), vectorToTest2.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransformCopy_Transform3D()
   {
      Random random = new Random(398742498237598750L);
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      Vector3D vectorToTransform = RandomGeometry.nextVector3D(random, 0.0, 0.0, 0.0, 100.0, 100.0, 0.0);
      FrameVector2D vectorToTest = new FrameVector2D(null, new Vector2D(vectorToTransform.getX(), vectorToTransform.getY())); 

      try
      {
         vectorToTest.applyTransform(transform);
         fail("Should have thrown RuntimeException");
      }
      catch(RuntimeException re)
      {
         //Good
      }

      double[] matrix = {6.0, 7.0, 0.0};
      RigidBodyTransform transform2 = EuclidCoreRandomTools.generateRandomRigidBodyTransform2D(random);

      Vector3D vectorToTransform2 = new Vector3D(matrix);
      FrameVector2D vectorToTest2 = new FrameVector2D(null, matrix);

      transform2.transform(vectorToTransform2);
      vectorToTest2.applyTransform(transform2);

      assertEquals("Should be equal", vectorToTransform2.getX(), vectorToTest2.getX(), epsilon);
      assertEquals("Should be equal", vectorToTransform2.getY(), vectorToTest2.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSets()
   {
      FrameVector3D alpha = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector3D beta = new FrameVector3D(ReferenceFrame.getWorldFrame(), 8.0, -2.0, 0.0);
      alpha.set(0.0, 0.0, 0.0);
      alpha.setX(-7.0);
      alpha.setY(10.3);
      alpha.setZ(1.9);
      alpha.set(10, 20, 30);
      alpha.setX(0);
      alpha.set(beta);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testadd()
   {
      FrameVector3D alpha = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector3D beta = new FrameVector3D(ReferenceFrame.getWorldFrame(), 7.0, 0.0, 6.0);
      alpha.add(beta);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInterpolate()
   {
      FrameVector3D alpha = new FrameVector3D(ReferenceFrame.getWorldFrame(), -1.0, 0.0, 17.0);
      FrameVector3D beta = new FrameVector3D(ReferenceFrame.getWorldFrame(), 3.3, 30.0, 9.0);
      FrameVector3D gamma = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 2.8, 3.0);

      gamma.interpolate(alpha, beta, 3.0);
      gamma.interpolate(beta, alpha, 1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetReferenceFrame()
   {
      FrameVector3D alpha = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector3D beta = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 20.0, 5.0);
      alpha.getReferenceFrame();
      beta.getReferenceFrame();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGets()
   {
      FrameVector3D alpha = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector3D beta = new FrameVector3D(ReferenceFrame.getWorldFrame(), 7.0, 0.0, -6.0);
      alpha.getX();
      beta.getY();
      beta.getZ();
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      FrameTuple3DReadOnlyTest.assertSuperMethodsAreOverloaded(FrameTuple2DReadOnly.class, Tuple2DReadOnly.class, FrameVector2D.class, Vector2DBasics.class);
   }
}
