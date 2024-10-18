package us.ihmc.robotics.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryMissingTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreMissingTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.log.LogTools;

public class GeometryToolsTest
{
   private static final int ITERATIONS = 1000;

   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private static final double EPSILON = 1e-6;



   @Test
   public void testClipToBoundingBox()
   {
      Tuple3DBasics tuple3d = new Point3D(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 0.1);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 2.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 1.0);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 1.0), tuple3d, 0.0);
   }

   @Test
   public void testCombine()
   {
      Random random = new Random(1176L);
      ArrayList<Point2D> firstList = new ArrayList<Point2D>();
      for (int i = 0; i < 100; i++)
      {
         firstList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D firstPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(firstList));

      ArrayList<Point2D> secondList = new ArrayList<Point2D>();
      for (int i = 0; i < 200; i++)
      {
         secondList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D secondPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(secondList));

      ConvexPolygon2D result = new ConvexPolygon2D(firstPolygon, secondPolygon);

      // convexity of the result is already checked in another test
      for (Point2D point : firstList)
      {
         if (!result.isPointInside(point))
         {
            double distance = result.distance(point);

            if (distance > 1e-7)
               throw new RuntimeException("Not each point is inside the result. distance = " + distance);
         }

         //       assertTrue("Not each point isinside the result. distance = " , result.isPointInside(point));
      }

      for (Point2D point : secondList)
      {
         if (!result.isPointInside(point))
         {
            double distance = result.distance(point);

            if (distance > 1e-7)
               throw new RuntimeException("Not each point is inside the result. distance = " + distance);
         }

         //       assertTrue("Not each point is inside the result", result.isPointInside(point));
      }
   }

   @Test
   public void testNormalizeSafeZUp() throws Exception
   {
      Vector3D actualVector;
      Vector3D expectedVector = new Vector3D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         actualVector = EuclidCoreRandomTools.nextVector3D(random, RandomNumbers.nextDouble(random, Epsilons.ONE_TRILLIONTH, 10.0));

         expectedVector.setAndNormalize(actualVector);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.999 * Epsilons.ONE_TRILLIONTH);
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = new Vector3D();
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);
      }
   }



   @Test
   public void testConstructFrameFromPointAndAxis()
   {
      Random random = new Random(1776L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePoint3D randomPoint = new FramePoint3D(worldFrame);

      FrameVector3D randomVector = new FrameVector3D(worldFrame);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         randomPoint.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0));
         randomVector.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0));

         ReferenceFrame frameA = GeometryTools.constructReferenceFrameFromPointAndZAxis("frameA", randomPoint, randomVector);
         ReferenceFrame frameB = GeometryTools.constructReferenceFrameFromPointAndAxis("frameB", randomPoint, Axis3D.Z, randomVector);

         EuclidCoreTestTools.assertEquals(frameA.getTransformToRoot(), frameB.getTransformToRoot(), 1.0e-2);
      }
   }

   @Test
   public void testYawAboutPointRegression()
   {
      double epsilon = 1e-10;
      // Do not change value! For regression.
      Random r = new Random(2899234L);

      ReferenceFrame referenceFrame;
      FramePoint3D pointToYawAbout;
      FramePoint3D point;
      double yaw;
      FramePoint3D result;

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(-2681.624165883151, result.getX(), epsilon, "not equal");
      assertEquals(-1528.2007328131492, result.getY(), epsilon, "not equal");
      assertEquals(2998.298763316407, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(2868.1077772133904, result.getX(), epsilon, "not equal");
      assertEquals(-3773.703916968001, result.getY(), epsilon, "not equal");
      assertEquals(-3313.247345650209, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(9865.290784196699, result.getX(), epsilon, "not equal");
      assertEquals(1276.040690119471, result.getY(), epsilon, "not equal");
      assertEquals(-3096.5574256022164, result.getZ(), epsilon, "not equal");
   }

   @Test
   public void testPitchAboutPointRegression()
   {
      double epsilon = 1e-10;
      // Do not change value! For regression.
      Random r = new Random(689291994L);

      ReferenceFrame referenceFrame;
      FramePoint3D pointToPitchAbout;
      FramePoint3D point;
      double pitch;
      FramePoint3D result;

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(-256.24551976827297, result.getX(), epsilon, "not equal");
      assertEquals(1443.7013411938358, result.getY(), epsilon, "not equal");
      assertEquals(11103.259343203952, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(-2273.346187036131, result.getX(), epsilon, "not equal");
      assertEquals(3010.5651766598717, result.getY(), epsilon, "not equal");
      assertEquals(-3513.344540982049, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(3978.4131392851787, result.getX(), epsilon, "not equal");
      assertEquals(682.5708442089929, result.getY(), epsilon, "not equal");
      assertEquals(8214.605434738955, result.getZ(), epsilon, "not equal");
   }

   @Test
   public void testYawAboutPoint()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      double epsilon = 1e-10;
      final FramePoint3D pointToYawAboutException = new FramePoint3D(theFrame, 0.0, 0.0, 0.0);
      final FramePoint3D pointException = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      final FramePoint3D resultException = new FramePoint3D();
      final double yawException = Math.PI;
      assertThrows(ReferenceFrameMismatchException.class,
                   () -> GeometryTools.yawAboutPoint(pointException, pointToYawAboutException, yawException, resultException));

      FramePoint3D pointToYawAbout = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      double yaw = Math.PI;

      FramePoint3D result = new FramePoint3D();
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      //      System.out.println(result);
      assertEquals(-1.0, result.getX(), epsilon, "These should be equal");
      assertEquals(-1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(1.0, result.getZ(), epsilon, "These should be equal");

      //Check for reference frame mismatch
      FramePoint3D point2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, point2);

      pointToYawAbout = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 1.0);
      yaw = Math.PI / 2;

      result = new FramePoint3D();
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      //      System.out.println(result);
      assertEquals(0.0, result.getX(), epsilon, "These should be equal");
      assertEquals(1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(1.0, result.getZ(), epsilon, "These should be equal");
   }

   @Test
   public void testPitchAboutPoint()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      double epsilon = 1e-10;
      final FramePoint3D pointToPitchAboutException = new FramePoint3D(theFrame, 0.0, 0.0, 0.0);
      final FramePoint3D pointException = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      final FramePoint3D resultException = new FramePoint3D();
      final double pitchException = Math.PI;
      assertThrows(ReferenceFrameMismatchException.class,
                   () -> GeometryTools.yawAboutPoint(pointException, pointToPitchAboutException, pitchException, resultException));

      FramePoint3D pointToPitchAbout = new FramePoint3D(theFrame, 0, 0, 0);
      FramePoint3D point = new FramePoint3D(theFrame, 1, 1, 1);
      double pitch = Math.PI;

      FramePoint3D result = new FramePoint3D();
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      //      System.out.println(result);
      assertEquals(-1.0, result.getX(), epsilon, "These should be equal");
      assertEquals(1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(-1.0, result.getZ(), epsilon, "These should be equal");
   }

   private double randomScalar(Random random)
   {
      return (random.nextDouble() - 0.5) * 10000.0;
   }

   private double randomAngle(Random random)
   {
      return (random.nextDouble() - 0.5) * 2.0 * Math.PI;
   }

   @Test
   public void testYawAboutPoint_FramePoint2d_double()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      ReferenceFrame aFrame = ReferenceFrameTools.constructARootFrame("aFrame");
      double epsilon = 1e-10;

      FramePoint2D original = new FramePoint2D(theFrame, 5.0, 7.0);
      FramePoint2D pointToYawAbout = new FramePoint2D(theFrame);
      double yaw = Math.PI;

      FramePoint2D result = new FramePoint2D(theFrame);
      GeometryTools.yawAboutPoint(original, pointToYawAbout, yaw, result);
      assertEquals(result.getX(), -original.getX(), epsilon, "Should be equal");
      assertEquals(result.getY(), -original.getY(), epsilon, "Should be equal");
      try
      {
         FramePoint2D pointToYawAbout2 = new FramePoint2D(aFrame);
         GeometryTools.yawAboutPoint(original, pointToYawAbout2, yaw, result);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }


   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(GeometryTools.class, GeometryToolsTest.class);
   }
}
