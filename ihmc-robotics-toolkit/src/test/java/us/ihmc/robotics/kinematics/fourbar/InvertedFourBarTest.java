package us.ihmc.robotics.kinematics.fourbar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Test;

import com.sun.javafx.application.PlatformImpl;

import javafx.application.Platform;
import javafx.beans.binding.Bindings;
import javafx.beans.binding.DoubleBinding;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Tests for {@link FourBar} using exclusively inverted configurations.
 */
public class InvertedFourBarTest
{
   static final double EPSILON = 1.0e-9;
   private static final double FD_DOT_EPSILON = 1.0e-4;
   private static final double FD_DDOT_EPSILON = 1.0e-2;
   private static final int ITERATIONS = 10000;
   static final boolean VERBOSE = false;

   @Test
   public void testAngleLimits() throws InterruptedException
   {
      Random random = new Random(67547);
      FourBar fourBar = new FourBar();

      for (int i = 0; i < ITERATIONS; i++)
      { // Generate convex points that are on a random circle, then flipping an edge.
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         List<Point2D> verticesNew = new ArrayList<>();
         verticesNew.addAll(Arrays.asList(new Point2D(), new Point2D(), new Point2D(), new Point2D()));

         UnitVector2D direction = new UnitVector2D();

         fourBar.setup(A, B, C, D);

         for (FourBarAngle angle : FourBarAngle.values)
         {
            fourBar.setup(A, B, C, D);
            fourBar.setToMin(angle);
            if (VERBOSE)
               System.out.println(fourBar);

            int startIndex = angle.ordinal();
            FourBarVertex vertex = fourBar.getVertex(angle);

            for (int j = 0; j < 4; j++)
               verticesNew.get(j).set(vertices.get(j));

            for (int j = 0; j < 2; j++)
            {
               Point2D vPrevNew = ListWrappingIndexTools.getPrevious(startIndex + j, verticesNew);
               Point2D vCurrNew = ListWrappingIndexTools.getWrap(startIndex + j, verticesNew);
               Point2D vNextNew = ListWrappingIndexTools.getNext(startIndex + j, verticesNew);

               direction.sub(vPrevNew, vCurrNew);
               RotationMatrixTools.applyYawRotation(vertex.getAngle(), direction, direction);
               vNextNew.scaleAdd(vertex.getNextEdge().getLength(), direction, vCurrNew);

               vertex = vertex.getNextVertex();
            }

            try
            {
               for (int j = 0; j < 4; j++)
               {
                  assertEquals(vertices.get(j).distance(vertices.get((j + 1) % 4)), verticesNew.get(j).distance(verticesNew.get((j + 1) % 4)), EPSILON);
               }

               assertEquals(0.0, fourBar.getAngleDAB() + fourBar.getAngleABC() + fourBar.getAngleBCD() + fourBar.getAngleCDA(), EPSILON);
            }
            catch (Throwable e)
            {
               Viewer viewer = startupViewer();
               viewer.updateFOV(A, B, C, D, verticesNew.get(0), verticesNew.get(1), verticesNew.get(2), verticesNew.get(3));
               draw(viewer, A, B, C, D);
               draw(viewer, verticesNew, "'");
               viewer.waitUntilClosed();

               throw e;
            }
         }
      }
   }

   @Test
   public void testAtLimits()
   {
      // Test the min/max configurations are continuous with configurations that are close to the limit.
      Random random = new Random(3465764);
      FourBar fourBar = new FourBar();

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);
         fourBar.setup(A, B, C, D);

         double expectedDAB, expectedABC, expectedBCD, expectedCDA;
         double angleLimitVariation = 1.0e-10;
         double tolerance = 1.0e-3;

         for (FourBarAngle fourBarAngle : FourBarAngle.values)
         {
            double minAngle = fourBar.getVertex(fourBarAngle).getMinAngle();
            double maxAngle = fourBar.getVertex(fourBarAngle).getMaxAngle();

            assertNull(fourBar.update(fourBarAngle, minAngle + angleLimitVariation)); // Assert that the given angle doesn't trigger limit edge-case
            expectedDAB = fourBar.getAngleDAB();
            expectedABC = fourBar.getAngleABC();
            expectedBCD = fourBar.getAngleBCD();
            expectedCDA = fourBar.getAngleCDA();

            fourBar.setToMin(fourBarAngle);
            assertEquals(expectedDAB, fourBar.getAngleDAB(), tolerance);
            assertEquals(expectedABC, fourBar.getAngleABC(), tolerance);
            assertEquals(expectedBCD, fourBar.getAngleBCD(), tolerance);
            assertEquals(expectedCDA, fourBar.getAngleCDA(), tolerance);

            assertEquals(Bound.MIN, fourBar.update(fourBarAngle, minAngle)); // Assert that the given angle triggers the limit
            assertEquals(expectedDAB, fourBar.getAngleDAB(), tolerance);
            assertEquals(expectedABC, fourBar.getAngleABC(), tolerance);
            assertEquals(expectedBCD, fourBar.getAngleBCD(), tolerance);
            assertEquals(expectedCDA, fourBar.getAngleCDA(), tolerance);

            assertNull(fourBar.update(fourBarAngle, maxAngle - angleLimitVariation)); // Assert that the given angle doesn't trigger limit edge-case
            expectedDAB = fourBar.getAngleDAB();
            expectedABC = fourBar.getAngleABC();
            expectedBCD = fourBar.getAngleBCD();
            expectedCDA = fourBar.getAngleCDA();

            fourBar.setToMax(fourBarAngle);
            assertEquals(expectedDAB, fourBar.getAngleDAB(), tolerance);
            assertEquals(expectedABC, fourBar.getAngleABC(), tolerance);
            assertEquals(expectedBCD, fourBar.getAngleBCD(), tolerance);
            assertEquals(expectedCDA, fourBar.getAngleCDA(), tolerance);

            assertEquals(Bound.MAX, fourBar.update(fourBarAngle, maxAngle)); // Assert that the given angle triggers the limit
            assertEquals(expectedDAB, fourBar.getAngleDAB(), tolerance);
            assertEquals(expectedABC, fourBar.getAngleABC(), tolerance);
            assertEquals(expectedBCD, fourBar.getAngleBCD(), tolerance);
            assertEquals(expectedCDA, fourBar.getAngleCDA(), tolerance);
         }
      }
   }

   @Test
   public void testGeometry() throws Throwable
   {
      Random random = new Random(345);
      FourBar fourBar = new FourBar();

      for (int i = 0; i < ITERATIONS; i++)
      { // Generate convex points that are on a random circle, then flipping an edge.
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);
         ConvexFourBarTest.performBasicGeometricAssertions(random, fourBar, i, A, B, C, D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Directly building a random inverted 4-bar
         Point2D A = new Point2D();
         Point2D B = new Point2D();
         Point2D C = new Point2D();
         Point2D D = new Point2D();

         // X is the intersection between the 2 diagonals
         Point2D X = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D XA = new Vector2D();
         Vector2D XB = new Vector2D();
         Vector2D XC = new Vector2D();
         Vector2D XD = new Vector2D();

         if (random.nextBoolean())
         { // Crossing edges: DA and BC
            UnitVector2D direction = EuclidCoreRandomTools.nextUnitVector2D(random);
            XA.setAndScale(+EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            XD.setAndScale(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0e-3, Math.PI - 1.0e-3);
            if (random.nextBoolean())
               yaw = -yaw;
            RotationMatrixTools.applyYawRotation(yaw, direction, direction);
            XB.setAndScale(+EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            XC.setAndScale(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
         }
         else
         { // Crossing edges: AB and CD
            UnitVector2D direction = EuclidCoreRandomTools.nextUnitVector2D(random);
            XA.setAndScale(+EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            XB.setAndScale(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0e-3, Math.PI - 1.0e-3);
            if (random.nextBoolean())
               yaw = -yaw;
            RotationMatrixTools.applyYawRotation(yaw, direction, direction);
            XD.setAndScale(+EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
            XC.setAndScale(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction);
         }

         A.add(X, XA);
         B.add(X, XB);
         C.add(X, XC);
         D.add(X, XD);
         ConvexFourBarTest.performBasicGeometricAssertions(random, fourBar, i, A, B, C, D);
      }
   }

   @Test
   public void testVelocityAgainstFiniteDifference()
   {
      Random random = new Random(4545786);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         FourBar fourBar = new FourBar();
         fourBar.setup(A, B, C, D);

         FourBarAngle fourBarAngle = EuclidCoreRandomTools.nextElementIn(random, FourBarAngle.values);

         double angleStart = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBar.getVertex(fourBarAngle).getMinAngle(),
                                                              fourBar.getVertex(fourBarAngle).getMaxAngle());
         fourBar.update(fourBarAngle, angleStart);
         double DAB_start = fourBar.getAngleDAB();
         double ABC_start = fourBar.getAngleABC();
         double BCD_start = fourBar.getAngleBCD();
         double CDA_start = fourBar.getAngleCDA();

         double AC_start = fourBar.getDiagonalAC().getLength();
         double BD_start = fourBar.getDiagonalBD().getLength();

         double angleEnd;
         double angleMaxDelta = 0.5e-7;
         double dt = 0.5e-6;

         if (random.nextBoolean())
         { // Going toward max
            angleMaxDelta = Math.min(angleMaxDelta, fourBar.getVertex(fourBarAngle).getMaxAngle() - angleStart);
            angleEnd = angleStart + EuclidCoreRandomTools.nextDouble(random, 0.0, angleMaxDelta);
         }
         else
         { // Going toward min
            angleMaxDelta = Math.max(-angleMaxDelta, fourBar.getVertex(fourBarAngle).getMinAngle() - angleStart);
            angleEnd = angleStart + EuclidCoreRandomTools.nextDouble(random, angleMaxDelta, 0.0);
         }

         fourBar.update(fourBarAngle, angleEnd);
         double DAB_end = fourBar.getAngleDAB();
         double ABC_end = fourBar.getAngleABC();
         double BCD_end = fourBar.getAngleBCD();
         double CDA_end = fourBar.getAngleCDA();

         double AC_end = fourBar.getDiagonalAC().getLength();
         double BD_end = fourBar.getDiagonalBD().getLength();

         double angleDot = (angleEnd - angleStart) / dt;

         double expected_dtDAB = (DAB_end - DAB_start) / dt;
         double expected_dtABC = (ABC_end - ABC_start) / dt;
         double expected_dtBCD = (BCD_end - BCD_start) / dt;
         double expected_dtCDA = (CDA_end - CDA_start) / dt;

         double expected_dtAC = (AC_end - AC_start) / dt;
         double expected_dtBD = (BD_end - BD_start) / dt;

         if (VERBOSE)
            System.out.printf("Expected:\n\tangleDots [DAB=%f, ABC=%f, BCD=%f, CDA=%f]\n\tdiagonal lengthDots [AC=%f, BD=%f]\n",
                              expected_dtDAB,
                              expected_dtABC,
                              expected_dtBCD,
                              expected_dtCDA,
                              expected_dtAC,
                              expected_dtBD);

         fourBar.update(fourBarAngle, angleStart, angleDot);

         assertEqualsDynEpsilon(expected_dtDAB, fourBar.getAngleDtDAB(), FD_DOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dtABC, fourBar.getAngleDtABC(), FD_DOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dtBCD, fourBar.getAngleDtBCD(), FD_DOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dtCDA, fourBar.getAngleDtCDA(), FD_DOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dtAC, fourBar.getDiagonalAC().getLengthDot(), FD_DOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dtBD, fourBar.getDiagonalBD().getLengthDot(), FD_DOT_EPSILON, "Iteration= " + i);
      }
   }

   @Test
   public void testAccelerationAgainstFiniteDifference()
   {
      Random random = new Random(4545786);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         FourBar fourBar = new FourBar();
         fourBar.setup(A, B, C, D);

         FourBarAngle fourBarAngle = FourBarAngle.DAB; //EuclidCoreRandomTools.nextElementIn(random, FourBarAngle.values);

         double dt = 1.0e-6;

         double angleStart = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBar.getVertex(fourBarAngle).getMinAngle(),
                                                              fourBar.getVertex(fourBarAngle).getMaxAngle());
         double angleEnd;
         double angleDotStart;
         double angleDotEnd;
         double angleDDot;
         do
         {
            double angleDotDelta = EuclidCoreRandomTools.nextDouble(random, 1.0e-6);
            angleDotStart = EuclidCoreRandomTools.nextDouble(random, 0.5);
            angleDotEnd = angleDotStart + angleDotDelta;
            angleDDot = angleDotDelta / dt;
            angleEnd = angleStart + (angleDotStart + 0.5 * angleDotDelta) * dt;
         }
         while (angleEnd > fourBar.getVertex(fourBarAngle).getMaxAngle() || angleEnd < fourBar.getVertex(fourBarAngle).getMinAngle());

         fourBar.update(fourBarAngle, angleStart, angleDotStart);
         double dtDAB_start = fourBar.getAngleDtDAB();
         double dtABC_start = fourBar.getAngleDtABC();
         double dtBCD_start = fourBar.getAngleDtBCD();
         double dtCDA_start = fourBar.getAngleDtCDA();

         double dtAC_start = fourBar.getDiagonalAC().getLengthDot();
         double dtBD_start = fourBar.getDiagonalBD().getLengthDot();

         fourBar.update(fourBarAngle, angleEnd, angleDotEnd);
         double dtDAB_end = fourBar.getAngleDtDAB();
         double dtABC_end = fourBar.getAngleDtABC();
         double dtBCD_end = fourBar.getAngleDtBCD();
         double dtCDA_end = fourBar.getAngleDtCDA();

         double dtAC_end = fourBar.getDiagonalAC().getLengthDot();
         double dtBD_end = fourBar.getDiagonalBD().getLengthDot();

         double expected_dt2DAB = (dtDAB_end - dtDAB_start) / dt;
         double expected_dt2ABC = (dtABC_end - dtABC_start) / dt;
         double expected_dt2BCD = (dtBCD_end - dtBCD_start) / dt;
         double expected_dt2CDA = (dtCDA_end - dtCDA_start) / dt;

         double expected_dt2AC = (dtAC_end - dtAC_start) / dt;
         double expected_dt2BD = (dtBD_end - dtBD_start) / dt;

         if (Math.abs(expected_dt2AC) > 1000.0 || Math.abs(expected_dt2BD) > 1000.0)
         {
            i--;
            continue;
         }

         if (VERBOSE)
            System.out.printf("Expected:\n\tangleDDots [DAB=%f, ABC=%f, BCD=%f, CDA=%f]\n\tdiagonal lengthDDots [AC=%f, BD=%f]\n",
                              expected_dt2DAB,
                              expected_dt2ABC,
                              expected_dt2BCD,
                              expected_dt2CDA,
                              expected_dt2AC,
                              expected_dt2BD);

         fourBar.update(fourBarAngle, angleStart, angleDotStart, angleDDot);

         assertEqualsDynEpsilon(expected_dt2DAB, fourBar.getAngleDt2DAB(), FD_DDOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dt2ABC, fourBar.getAngleDt2ABC(), FD_DDOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dt2BCD, fourBar.getAngleDt2BCD(), FD_DDOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dt2CDA, fourBar.getAngleDt2CDA(), FD_DDOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dt2AC, fourBar.getDiagonalAC().getLengthDDot(), FD_DDOT_EPSILON, "Iteration= " + i);
         assertEqualsDynEpsilon(expected_dt2BD, fourBar.getDiagonalBD().getLengthDDot(), FD_DDOT_EPSILON, "Iteration= " + i);
      }
   }

   private void assertEqualsDynEpsilon(double expected, double actual, double epsilon, String messagePrefix)
   {
      double dynEspilon = epsilon * Math.max(1.0, Math.abs(expected));
      String errorMessage = "rror= " + +Math.abs(expected - actual) + ", epsilon= " + dynEspilon;
      if (messagePrefix == null || messagePrefix.isEmpty())
         messagePrefix = "E" + errorMessage;
      else
         messagePrefix += ", e" + errorMessage;

      assertEquals(expected, actual, dynEspilon, messagePrefix);
   }

   @Test
   public void testAccelerationsWithRandomQuadrilateral()
   {
      double eps = 1.0e-5;
      Random random = new Random(1984L);
      FourBar fourBar;

      double DAB_t0 = 0.0, DAB_tf = 0.0, DAB_tj = 0.0;
      double dDAB_t0 = 0.0, dDAB_tf = 0.0, dDAB_tj = 0.0;
      double ddDAB = 0.0;

      double ABC_t0 = 0.0, CDA_t0 = 0.0, BCD_t0 = 0.0;
      double ABC_next_tj = 0.0, CDA_next_tj = 0.0, BCD_next_tj = 0.0;
      double ABC_numerical_tf = 0.0, CDA_numerical_tf = 0.0, BCD_numerical_tf = 0.0;
      double ABC_fourbar_tf = 0.0, CDA_fourbar_tf = 0.0, BCD_fourbar_tf = 0.0;

      int nSteps = 10000;
      double T = 0.001, deltaT = T / (nSteps - 1.0);

      for (int i = 0; i < 50; i++)
      {
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         fourBar = new FourBar();
         fourBar.setup(A, B, C, D);

         DAB_t0 = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         DAB_tf = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         dDAB_t0 = 0.0;
         ddDAB = 2.0 * (DAB_tf - DAB_t0 - dDAB_t0 * T) / MathTools.square(T);
         dDAB_tf = dDAB_t0 + ddDAB * T;

         fourBar.update(FourBarAngle.DAB, DAB_t0, dDAB_t0);
         ABC_t0 = fourBar.getAngleABC();
         CDA_t0 = fourBar.getAngleCDA();
         BCD_t0 = fourBar.getAngleBCD();

         fourBar.update(FourBarAngle.DAB, DAB_tf, dDAB_tf);
         ABC_fourbar_tf = fourBar.getAngleABC();
         CDA_fourbar_tf = fourBar.getAngleCDA();
         BCD_fourbar_tf = fourBar.getAngleBCD();

         ABC_next_tj = ABC_t0;
         CDA_next_tj = CDA_t0;
         BCD_next_tj = BCD_t0;

         for (int j = 0; j < nSteps - 1; j++)
         {
            DAB_tj = ddDAB * MathTools.square(j * deltaT) / 2.0 + dDAB_t0 * j * deltaT + DAB_t0;
            dDAB_tj = ddDAB * j * deltaT + dDAB_t0;
            fourBar.update(FourBarAngle.DAB, DAB_tj, dDAB_tj, ddDAB);

            ABC_next_tj += fourBar.getAngleDtABC() * deltaT + fourBar.getAngleDt2ABC() * MathTools.square(deltaT) / 2.0;
            CDA_next_tj += fourBar.getAngleDtCDA() * deltaT + fourBar.getAngleDt2CDA() * MathTools.square(deltaT) / 2.0;
            BCD_next_tj += fourBar.getAngleDtBCD() * deltaT + fourBar.getAngleDt2BCD() * MathTools.square(deltaT) / 2.0;
         }

         ABC_numerical_tf = ABC_next_tj;
         CDA_numerical_tf = CDA_next_tj;
         BCD_numerical_tf = BCD_next_tj;

         assertEquals(ABC_numerical_tf, ABC_fourbar_tf, eps);
         assertEquals(CDA_numerical_tf, CDA_fourbar_tf, eps);
         assertEquals(BCD_numerical_tf, BCD_fourbar_tf, eps);
      }
   }

   public static class Viewer
   {
      double span = 10.0;
      Point2D center = new Point2D();

      Scene scene;
      Group root;
      final CountDownLatch countDownLatch = new CountDownLatch(1);

      public void updateFOV(List<? extends Point2DReadOnly> points)
      {
         updateFOV(points.toArray(new Point2DReadOnly[0]));
      }

      public void updateFOV(Point2DReadOnly... points)
      {
         BoundingBox2D bbx = new BoundingBox2D();
         bbx.updateToIncludePoints(Vertex2DSupplier.asVertex2DSupplier(points));
         span = 1.2 * Math.max(bbx.getMaxX() - bbx.getMinX(), bbx.getMaxY() - bbx.getMinY());
         bbx.getCenterPoint(center);
      }

      public void waitUntilClosed() throws InterruptedException
      {
         countDownLatch.await();
      }

      public DoubleBinding yPositionProperty(double span, Point2D center, Scene scene, Point2DReadOnly vertex)
      {
         return Bindings.multiply(0.5 - (vertex.getY() - center.getY()) / span, scene.heightProperty());
      }

      public DoubleBinding xPositionProperty(double span, Point2D center, Scene scene, Point2DReadOnly vertex)
      {
         return Bindings.multiply(0.5 + (vertex.getX() - center.getX()) / span, scene.widthProperty());
      }
   }

   public static Viewer startupViewer()
   {
      Viewer viewerComponents = new Viewer();

      PlatformImpl.startup(() ->
      {
         Stage stage = new Stage();
         viewerComponents.root = new Group();
         viewerComponents.scene = new Scene(viewerComponents.root, 600, 600);

         stage.setScene(viewerComponents.scene);
         stage.show();
         stage.setOnCloseRequest(e -> viewerComponents.countDownLatch.countDown());
      });

      return viewerComponents;
   }

   private static class VertexGraphics implements Point2DReadOnly
   {
      private Point2DReadOnly vertex;
      private String name;
      private Color color;

      public VertexGraphics(Point2DReadOnly vertex, String name, Color color)
      {
         this.vertex = vertex;
         this.name = name;
         this.color = color;
      }

      @Override
      public double getX()
      {
         return vertex.getX();
      }

      @Override
      public double getY()
      {
         return vertex.getY();
      }
   }

   public static void draw(Viewer viewer, List<? extends Point2DReadOnly> vertices) throws InterruptedException
   {
      draw(viewer, vertices, "");
   }

   public static void draw(Viewer viewer, Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D) throws InterruptedException
   {
      draw(viewer, A, B, C, D, "");
   }

   public static void draw(Viewer viewer, List<? extends Point2DReadOnly> vertices, String suffix) throws InterruptedException
   {
      draw(viewer, vertices.get(0), vertices.get(1), vertices.get(2), vertices.get(3), suffix);
   }

   public static void draw(Viewer viewer, Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D, String suffix) throws InterruptedException
   {
      VertexGraphics AGraphics = new VertexGraphics(A, "A" + suffix, Color.INDIANRED);
      VertexGraphics BGraphics = new VertexGraphics(B, "B" + suffix, Color.CORNFLOWERBLUE);
      VertexGraphics CGraphics = new VertexGraphics(C, "C" + suffix, Color.DARKGREEN);
      VertexGraphics DGraphics = new VertexGraphics(D, "D" + suffix, Color.GOLD);
      draw(viewer, new VertexGraphics[] {AGraphics, BGraphics, CGraphics, DGraphics});
   }

   public static void draw(Viewer viewer, VertexGraphics... vertices) throws InterruptedException
   {
      Platform.runLater(() ->
      {
         Group root = viewer.root;
         Scene scene = viewer.scene;

         for (int i = 0; i < 4; i++)
         {
            VertexGraphics vertex = vertices[i];
            VertexGraphics nextVertex = vertices[(i + 1) % 4];
            VertexGraphics previousVertex = vertices[(i + 3) % 4];
            boolean moveRight = vertex.getX() > previousVertex.getX() && vertex.getX() > nextVertex.getX();
            boolean moveLeft = vertex.getX() < previousVertex.getX() && vertex.getX() < nextVertex.getX();
            boolean moveTop = vertex.getY() > previousVertex.getY() && vertex.getY() > nextVertex.getY();
            boolean moveBottom = vertex.getY() < previousVertex.getY() && vertex.getY() < nextVertex.getY();

            Text text = new Text(vertex.name);
            text.setStroke(vertex.color);
            DoubleBinding xPositionProperty = viewer.xPositionProperty(viewer.span, viewer.center, scene, vertex);
            DoubleBinding yPositionProperty = viewer.yPositionProperty(viewer.span, viewer.center, scene, vertex);
            if (moveLeft)
               xPositionProperty = xPositionProperty.subtract(15.0);
            else if (moveRight)
               xPositionProperty = xPositionProperty.add(10.0);
            text.xProperty().bind(xPositionProperty);

            if (moveTop)
               yPositionProperty = yPositionProperty.subtract(10.0);
            else if (moveBottom)
               yPositionProperty = yPositionProperty.add(15.0);
            text.yProperty().bind(yPositionProperty);
            root.getChildren().add(text);
         }

         for (int i = 0; i < 4; i++)
         {
            VertexGraphics vertex = vertices[i];
            VertexGraphics nextVertex = vertices[(i + 1) % 4];

            Line line = new Line();
            line.startXProperty().bind(viewer.xPositionProperty(viewer.span, viewer.center, scene, vertex));
            line.startYProperty().bind(viewer.yPositionProperty(viewer.span, viewer.center, scene, vertex));
            line.endXProperty().bind(viewer.xPositionProperty(viewer.span, viewer.center, scene, nextVertex));
            line.endYProperty().bind(viewer.yPositionProperty(viewer.span, viewer.center, scene, nextVertex));
            line.setStrokeWidth(2.0);
            line.setStroke(vertex.color.interpolate(nextVertex.color, 0.5));
            root.getChildren().add(line);
         }

         for (int i = 0; i < 4; i++)
         {
            VertexGraphics vertex = vertices[i];

            Circle circle = new Circle(5.0);
            circle.centerXProperty().bind(viewer.xPositionProperty(viewer.span, viewer.center, scene, vertex));
            circle.centerYProperty().bind(viewer.yPositionProperty(viewer.span, viewer.center, scene, vertex));
            circle.setFill(vertex.color);
            root.getChildren().add(circle);
         }
      });
   }
}
