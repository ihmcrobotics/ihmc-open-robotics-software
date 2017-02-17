package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameTuple2dTest<T extends FrameTuple2d<?, ?>>
{
   protected static final boolean VERBOSE = false;

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame frameTransformInPlane;
   private ReferenceFrame frameTransformNOTInPlane;

   protected static final double epsilon = 1e-10;

   protected ReferenceFrame theFrame = ReferenceFrame.constructARootFrame("theFrame", false, true, true);

   protected ReferenceFrame aFrame = ReferenceFrame.constructARootFrame("aFrame", false, true, true);

   protected RigidBodyTransform theFrameToChildFrame;

   protected ReferenceFrame childFrame;

   public abstract T createFrameTuple(ReferenceFrame referenceFrame, double x, double y, String name);

   public T createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return createFrameTuple(referenceFrame, x, y, null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testChangeFrame()
   {
      final RigidBodyTransform transformInPlane = new RigidBodyTransform();
      final RigidBodyTransform transformNOTInPlane = new RigidBodyTransform();
      frameTransformInPlane = new ReferenceFrame("frameTransformInPlane", worldFrame)
      {
         private static final long serialVersionUID = 5529935689015384902L;

         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformInPlane);
         }
      };
      frameTransformNOTInPlane = new ReferenceFrame("frameTransformNOTInPlane", worldFrame)
      {
         private static final long serialVersionUID = 5529935689015384902L;

         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformNOTInPlane);
         }
      };

      Random random = new Random(251654165L);
      T framePoint2d = createFrameTuple(worldFrame, 2.0, 4.0);

      for (int i = 0; i < 100000; i++)
      {
         transformInPlane.setIdentity();
         transformInPlane.setRotationYawAndZeroTranslation(RandomTools.generateRandomDouble(random, -Math.PI, Math.PI));
         transformInPlane.setTranslation(RandomTools.generateRandomVector(random));

         transformNOTInPlane.setIdentity();
         transformNOTInPlane.set(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));

         frameTransformInPlane.update();
         frameTransformNOTInPlane.update();

         Point2D point2d = RandomTools.generateRandomPoint2d(random, 1.0, 1.0);

         framePoint2d.setIncludingFrame(worldFrame, point2d);
         try
         {
            framePoint2d.changeFrame(frameTransformInPlane);
         }
         catch (RuntimeException e)
         {
            e.printStackTrace();
            System.out.println("Iteration: " + i);
            System.out.println(transformInPlane);
            fail("Should NOT have thrown a RuntimeException");
         }

         framePoint2d.setIncludingFrame(worldFrame, point2d);
         try
         {
            framePoint2d.changeFrame(frameTransformNOTInPlane);
            System.out.println("Iteration: " + i);
            System.out.println(transformNOTInPlane);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // Good
         }
      }
   }

   @Before
   public final void setUp() throws Exception
   {
      theFrameToChildFrame = new RigidBodyTransform();
      childFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", theFrame, theFrameToChildFrame, false, true, true);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSet_doubledouble()
   {
      FrameTuple2d<?, ?> doubleDouble = createFrameTuple(frameTransformInPlane, 0.0, 0.0);
      double[] expecteds1 = {-10.0, 10.0};
      doubleDouble.set(expecteds1[0], expecteds1[1]);
      double[] actuals1 = {doubleDouble.getX(), doubleDouble.getY()};
      assertArrayEquals(expecteds1, actuals1, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetIncludingFrame_ReferenceFrame_double_double()
   {
      Random random = new Random(456456456L);
      FrameTuple2d<?, ?> doubleDouble = createFrameTuple(theFrame, random.nextDouble(), random.nextDouble());
      double[] expecteds2 = {random.nextDouble(), random.nextDouble()}; 
      doubleDouble.setIncludingFrame(aFrame, expecteds2[0], expecteds2[1]);
      double[] actuals2 = {doubleDouble.getX(), doubleDouble.getY()};
      assertArrayEquals(expecteds2, actuals2, epsilon);
      doubleDouble.checkReferenceFrameMatch(aFrame);

      try
      {
         doubleDouble.checkReferenceFrameMatch(theFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSet_FrameTuple2d()
   {
      double[] expecteds1 = {-10.0, 10.0};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, 0.0, 0.0);
      FrameTuple2d<?, ?> sameFrame = createFrameTuple(theFrame, expecteds1[0], expecteds1[1]);
      original.set(sameFrame);
      double[] results1 = {original.getX(), original.getY()};
      
      original.checkReferenceFrameMatch(theFrame);

      try
      {
         original.checkReferenceFrameMatch(aFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      
      assertArrayEquals(expecteds1, results1, epsilon);

      try
      {
         FrameTuple2d<?, ?> differentFrame = createFrameTuple(aFrame, expecteds1[0], expecteds1[1]);
         original.set(differentFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetIncludingFrame_FrameTuple2d()
   {
      double[] expecteds1 = {-10.0, 10.0};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, 0.0, 0.0);
      FrameTuple2d<?, ?> modified = createFrameTuple(aFrame, expecteds1[0], expecteds1[1]);
      original.setIncludingFrame(modified);
      double[] results1 = {original.getX(), original.getY()};

      original.checkReferenceFrameMatch(modified.getReferenceFrame());

      try
      {
         original.checkReferenceFrameMatch(theFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      
      assertArrayEquals(expecteds1, results1, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetX_double()
   {
      double x = -123678346.56756;
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 0.0, 0.0);
      frameTuple.setX(x);
      assertEquals("These should be equal", x, frameTuple.getX(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetY_double()
   {
      double y = -123678346.56756;
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 0.0, 0.0);
      frameTuple.setY(y);
      assertEquals("These should be equal", y, frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScale_double()
   {
      double scaleFactor = 2.0;
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 2.0, 2.0);
      double x = scaleFactor * frameTuple.getX();
      double y = scaleFactor * frameTuple.getY();
      frameTuple.scale(scaleFactor);
      assertEquals("These should be equal", x, frameTuple.getX(), epsilon);
      assertEquals("These should be equal", y, frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScale_doubledouble()
   {
      double scaleXFactor = 2.0;
      double scaleYFactor = 3.0;
      double x = scaleXFactor * 2, y = scaleYFactor * 2;
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 2.0, 2.0);
      frameTuple.scale(scaleXFactor, scaleYFactor);
      assertEquals("These should be equal", x, frameTuple.getX(), epsilon);
      assertEquals("These should be equal", y, frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetPointCopy()
   {
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 10.0, 10.0);
      Point2D point = frameTuple.getPointCopy();
      assertEquals("These should be equal", point.getX(), frameTuple.getX(), epsilon);
      assertEquals("These should be equal", point.getY(), frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVectorCopy()
   {
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 10.0, 10.0);
      Vector2D point = frameTuple.getVectorCopy();
      assertEquals("These should be equal", point.getX(), frameTuple.getX(), epsilon);
      assertEquals("These should be equal", point.getY(), frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet_Tuple2d()
   {
      Point2D tuple2dToPack = new Point2D();
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      frameTuple.get(tuple2dToPack);
      assertEquals("These should be equal", values[0], tuple2dToPack.getX(), epsilon);
      assertEquals("These should be equal", values[1], tuple2dToPack.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet_Tuple3d()
   {
      Point3D tuple3dToPack = new Point3D();
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      frameTuple.get(tuple3dToPack);
      assertEquals("These should be equal", values[0], tuple3dToPack.getX(), epsilon);
      assertEquals("These should be equal", values[1], tuple3dToPack.getY(), epsilon);
      assertEquals("These should be equal", 0.0, tuple3dToPack.getZ(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetToZero()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      assertEquals("These should be equal", values[0], frameTuple.getX(), epsilon);
      assertEquals("These should be equal", values[1], frameTuple.getY(), epsilon);
      frameTuple.setToZero();
      assertEquals("These should be equal", 0.0, frameTuple.getX(), epsilon);
      assertEquals("These should be equal", 0.0, frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetToZero_ReferenceFrame()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      assertEquals("These should be equal", values[0], frameTuple.getX(), epsilon);
      assertEquals("These should be equal", values[1], frameTuple.getY(), epsilon);
      frameTuple.setToZero(theFrame);
      assertEquals("These should be equal", 0.0, frameTuple.getX(), epsilon);
      assertEquals("These should be equal", 0.0, frameTuple.getY(), epsilon);
      
      frameTuple.checkReferenceFrameMatch(theFrame);

      try
      {
         frameTuple.checkReferenceFrameMatch(aFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetToNaN()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      assertFalse(Double.isNaN(frameTuple.getX()));
      assertFalse(Double.isNaN(frameTuple.getY()));
      frameTuple.setToNaN();      
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetToNaN_ReferenceFrame()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      
      assertFalse(Double.isNaN(frameTuple.getX()));
      assertFalse(Double.isNaN(frameTuple.getY()));
      frameTuple.checkReferenceFrameMatch(aFrame);
      
      frameTuple.setToNaN(theFrame);      
      
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
      
      frameTuple.checkReferenceFrameMatch(theFrame);

      try
      {
         frameTuple.checkReferenceFrameMatch(aFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCheckForNaN()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, values[0], values[1]);
      FrameTuple2d<?, ?> frameTupleNaN = createFrameTuple(aFrame, values[0], values[1]);
      frameTupleNaN.setToNaN();

      frameTuple.checkForNaN();
      try
      {
         frameTupleNaN.checkForNaN();
         fail("Threw RuntimeException");
      }
      catch(RuntimeException re)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testContainsNaN()
   {
      double[] values = {456465.067, 456.898};
      FrameTuple2d<?, ?> neitherNaN = createFrameTuple(aFrame, values[0], values[1]);
      FrameTuple2d<?, ?> xNaN = createFrameTuple(aFrame, Double.NaN, values[1]);
      FrameTuple2d<?, ?> yNaN = createFrameTuple(aFrame, values[0], Double.NaN);
      FrameTuple2d<?, ?> bothNaN = createFrameTuple(aFrame, values[0], values[1]);
      bothNaN.setToNaN();

      assertFalse(neitherNaN.containsNaN());
      assertTrue(xNaN.containsNaN());
      assertTrue(yNaN.containsNaN());
      assertTrue(bothNaN.containsNaN());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScale_double_Tuple2d()
   {
      double[] values = {8.0, -5.0};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, 0.0, 0.0);
      double scaleFactor = 7.0;
      Point2D tuple2d = new Point2D(values);
      frameTuple.scale(scaleFactor, tuple2d);

      assertEquals("Should be equal doubles", scaleFactor * values[0], frameTuple.getX(), epsilon);
      assertEquals("Should be equal doubles", scaleFactor * values[1], frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScaleAdd_double_Tuple2d_Tuple2d()
   {
      double[] values1 = {8.0, -5.0};
      double[] values2 = {19.6, -3.9};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, 0.0, 0.0);
      double scaleFactor = -7.43;
      Point2D tuple1 = new Point2D(values1);
      Point2D tuple2 = new Point2D(values2);

      frameTuple.scaleAdd(scaleFactor, tuple1, tuple2);      
      assertEquals("Should be equal doubles", scaleFactor * values1[0] + values2[0], frameTuple.getX(), epsilon);
      assertEquals("Should be equal doubles", scaleFactor * values1[1] + values2[1], frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScaleAdd_double_Tuple2d_double_Tuple2d()
   {
      double[] values1 = {8.0, -5.0};
      double[] values2 = {19.6, -3.9};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, 1.5, 7.0);
      double scaleFactor1 = -7.43, scaleFactor2 = 6.545;
      Point2D tuple1 = new Point2D(values1);
      Point2D tuple2 = new Point2D(values2);

      frameTuple.scaleAdd(scaleFactor1, tuple1, scaleFactor2, tuple2);      
      assertEquals("Should be equal doubles", scaleFactor1 * values1[0] + scaleFactor2 * values2[0], frameTuple.getX(), epsilon);
      assertEquals("Should be equal doubles", scaleFactor1 * values1[1] + scaleFactor2 * values2[1], frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScaleAdd_double_Tuple2d()
   {
      double[] tuple2dValues = {8.0, 5.0};
      double[] frameTuple2dValues = {6.4, 3.9};
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(aFrame, frameTuple2dValues[0], frameTuple2dValues[1]);
      double scaleFactor1 = 7.43;
      Point2D tuple1 = new Point2D(tuple2dValues);

      frameTuple.scaleAdd(scaleFactor1, tuple1);      
      assertEquals("Should be equal doubles", scaleFactor1 * frameTuple2dValues[0] + tuple2dValues[0], frameTuple.getX(), epsilon);
      assertEquals("Should be equal doubles", scaleFactor1 * frameTuple2dValues[1] + tuple2dValues[1], frameTuple.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScale_double_FrameTuple2d()
   {
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, 1.0, 2.0);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, 3.0, 4.0);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, 5.0, 6.0);
      double scaleFactor = 7.0;

      original.scale(scaleFactor, same);
      assertEquals("These shoud be equal", original.getX(), scaleFactor * same.getX(), epsilon);
      assertEquals("These shoud be equal", original.getY(), scaleFactor * same.getY(), epsilon);

      try
      {
         original.scale(scaleFactor, different);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScaleAdd_double_FrameTuple2d_FrameTuple2d()
   {
      double[] originalValues = {1.0, 2.0};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, 3.0, 4.0);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, 5.0, 6.0);
      double scaleFactor = 7.0;

      original.scaleAdd(scaleFactor, original, same);
      assertEquals("These shoud be equal", original.getX(), scaleFactor * originalValues[0] + same.getX(), epsilon);
      assertEquals("These shoud be equal", original.getY(), scaleFactor * originalValues[1] + same.getY(), epsilon);

      try
      {
         original.scaleAdd(scaleFactor, original, different);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         original.scaleAdd(scaleFactor, different, original);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScaleAdd_double_FrameTuple2d()
   {
      double scaleFactor = 9.78;
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      original.scaleAdd(scaleFactor, same);
      assertEquals(scaleFactor * originalValues[0] + same.getX(), original.getX(), epsilon);
      assertEquals(scaleFactor * originalValues[1] + same.getY(), original.getY(), epsilon);
      try
      {
         original.scaleAdd(scaleFactor, different);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAdd_double_Tuple2d_Tuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      Point2D tuple1 = new Point2D(1.6, 43.6);
      Point2D tuple2 = new Point2D(5.665, 34.7);
      original.add(tuple1, tuple2);
      assertEquals(original.getX(), tuple1.getX() + tuple2.getX(), epsilon);
      assertEquals(original.getY(), tuple1.getY() + tuple2.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAdd_FrameTuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      original.add(same);
      assertEquals(originalValues[0] + originalValues[0], original.getX(), epsilon);
      assertEquals(originalValues[1] + originalValues[1], original.getY(), epsilon);
      try
      {
         original.add(different);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAdd_FrameTuple2d_FrameTuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      original.add(original, same);
      assertEquals(originalValues[0] + originalValues[0], original.getX(), epsilon);
      assertEquals(originalValues[1] + originalValues[1], original.getY(), epsilon);
      try
      {
         original.add(original, different);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         original.add(different, original);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSub_Tuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      Point2D tuple1 = new Point2D(originalValues[0], originalValues[1]);
      original.sub(tuple1);
      assertEquals("These should be equal: ", originalValues[0] - tuple1.getX(), original.getX(), epsilon);
      assertEquals("These should be equal: ", originalValues[1] - tuple1.getY(), original.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSub_Tuple2d_Tuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      Point2D tuple1 = new Point2D(originalValues[0], originalValues[1]);
      Point2D tuple2 = new Point2D(originalValues[0], originalValues[1]);

      original.sub(tuple1, tuple2);
      assertEquals("These should be equal: ", tuple1.getX() - tuple2.getX(), original.getX(), epsilon);
      assertEquals("These should be equal: ", tuple1.getY() - tuple2.getY(), original.getY(), epsilon);
      
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      try
      {
         original.sub(original, different);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         original.sub(different, original);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSub_FrameTuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      original.sub(same);
      assertEquals(originalValues[0] - originalValues[0], original.getX(), epsilon);
      assertEquals(originalValues[1] - originalValues[1], original.getY(), epsilon);
      try
      {
         original.sub(different);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSub_FrameTuple2d_FrameTuple2d()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, originalValues[0], originalValues[1]);

      original.sub(original, same);
      assertEquals(originalValues[0] - originalValues[0], original.getX(), epsilon);
      assertEquals(originalValues[1] - originalValues[1], original.getY(), epsilon);
      try
      {
         original.sub(original, different);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         original.sub(different, original);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInterpolate_Tuple2d_Tuple2d_double()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      Point2D tuple1 = new Point2D(6.7, 45.6);
      Point2D tuple2 = new Point2D(56.6, 3.5);
      double alpha = 2.0;

      original.interpolate(tuple1, tuple2, alpha);
      assertEquals("Should be equal", (1-alpha) * tuple1.getX() + alpha * tuple2.getX(), original.getX(), epsilon);
      assertEquals("Should be equal", (1-alpha) * tuple1.getY() + alpha * tuple2.getY(), original.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInterpolate_FrameTuple2d_FrameTuple2d_double()
   {
      double[] originalValues = {1.9, 3.7};
      double[] sameValues = {0.4, 2.6};
      double[] differentValues = {2.7, 4.0};
      double alpha = 2.7;
      FrameTuple2d<?, ?> holder = createFrameTuple(aFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> original = createFrameTuple(theFrame, originalValues[0], originalValues[1]);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, sameValues[0], sameValues[1]);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, differentValues[0], differentValues[1]);

      holder.interpolate(original, same, alpha);
      assertEquals("Should be equal", (1-alpha) * original.getX() + alpha * same.getX(), holder.getX(), epsilon);
      assertEquals("Should be equal", (1-alpha) * original.getY() + alpha * same.getY(), holder.getY(), epsilon);
      
      holder.checkReferenceFrameMatch(original.getReferenceFrame());

      try
      {
         holder.checkReferenceFrameMatch(aFrame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }

      try
      {
         holder.interpolate(original, different, alpha);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testClipToMinMax_double_double()
   {
      double minValue = -5.0;
      double maxValue = 5.0;
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, -34345.5455, 45456.45456);
      frameTuple.clipToMinMax(minValue, maxValue);
      assertTrue(frameTuple.getX() >= minValue && frameTuple.getX() <= maxValue);
      assertTrue(frameTuple.getY() >= minValue && frameTuple.getY() <= maxValue); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegate()
   {
      double[] originalValues = {1.9, 3.7};
      FrameTuple2d<?, ?> holder = createFrameTuple(aFrame, originalValues[0], originalValues[1]);
      holder.negate();
      assertEquals("Should be negated", - originalValues[0], holder.getX(), epsilon);
      assertEquals("Should be negated", - originalValues[1], holder.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEpsilonEquals_Tuple2d_double()
   {
      double threshold = 10.0;
      Point2D tuple1 = new Point2D(10.0, 10.0);
      Point2D tuple2 = new Point2D(11.0, 11.0);
      Point2D tuple3 = new Point2D(11.1, 11.1);
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 1.0);
      assertTrue(frameTuple.epsilonEquals(tuple1, threshold));
      assertTrue(frameTuple.epsilonEquals(tuple2, threshold));
      assertFalse(frameTuple.epsilonEquals(tuple3, threshold));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEpsilonEquals_FrameTuple2d_double()
   {
      double threshold = 10.0;
      FrameTuple2d<?, ?> tuple1 = createFrameTuple(theFrame, 10.0, 10.0);
      FrameTuple2d<?, ?> tuple2 = createFrameTuple(theFrame, 11.0, 11.0);
      FrameTuple2d<?, ?> tuple3 = createFrameTuple(theFrame, 11.1, 11.1);

      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 1.0);
      assertTrue(frameTuple.epsilonEquals(tuple1, threshold));
      assertTrue(frameTuple.epsilonEquals(tuple2, threshold));
      assertFalse(frameTuple.epsilonEquals(tuple3, threshold));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCheckReferenceFrameMatch_ReferenceFrameHolder()
   {
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 1.0);
      FrameTuple2d<?, ?> same = createFrameTuple(theFrame, 1.0, 1.0);
      FrameTuple2d<?, ?> different = createFrameTuple(aFrame, 1.0, 1.0);
      frameTuple.checkReferenceFrameMatch(same);
      try
      {
         frameTuple.checkReferenceFrameMatch(different);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCheckReferenceFrameMatch_ReferenceFrame()
   {
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 1.0);
      frameTuple.checkReferenceFrameMatch(theFrame);
      try
      {
         frameTuple.checkReferenceFrameMatch(aFrame);
         fail();
      }
      catch(ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToArray()
   {
      FrameTuple2d<?, ?> frameTuple = createFrameTuple(theFrame, 10.0, 10.0);
      double[] array = frameTuple.toArray();
      assertEquals("Should be equal", frameTuple.getX(), array[0], epsilon);
      assertEquals("Should be equal", frameTuple.getY(), array[1], epsilon);

   }
}
