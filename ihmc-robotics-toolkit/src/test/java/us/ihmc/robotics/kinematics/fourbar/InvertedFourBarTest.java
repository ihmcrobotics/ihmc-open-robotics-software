package us.ihmc.robotics.kinematics.fourbar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Test;

import com.sun.javafx.application.PlatformImpl;

import javafx.beans.binding.Bindings;
import javafx.beans.binding.DoubleBinding;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class InvertedFourBarTest
{
   private static final double EPSILON = 1.0e-9;
   private static final int ITERATIONS = 10000;
   private static final boolean VERBOSE = false;

   @Test
   public void testAngleLimits() throws InterruptedException
   {
      Random random = new Random(67547);
      InvertedFourBar fourBar = new InvertedFourBar();

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

         for (FourBarAngle angle : FourBarAngle.values())
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
   public void testGeometry() throws Throwable
   {
      Random random = new Random(345);
      InvertedFourBar fourBar = new InvertedFourBar();

      for (int i = 0; i < ITERATIONS; i++)
      { // Generate convex points that are on a random circle, then flipping an edge.
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);
         performBasicGeometricAssertions(random, fourBar, i, A, B, C, D);
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
         performBasicGeometricAssertions(random, fourBar, i, A, B, C, D);
      }
   }

   public void performBasicGeometricAssertions(Random random, InvertedFourBar fourBar, int iteration, Point2D A, Point2D B, Point2D C, Point2D D)
         throws InterruptedException, Throwable
   {
      fourBar.setup(A, B, C, D);

      Vector2D AB = new Vector2D();
      Vector2D BC = new Vector2D();
      Vector2D CD = new Vector2D();
      Vector2D DA = new Vector2D();

      AB.sub(B, A);
      BC.sub(C, B);
      CD.sub(D, C);
      DA.sub(A, D);

      Vector2D BA = new Vector2D(AB);
      Vector2D CB = new Vector2D(BC);
      Vector2D DC = new Vector2D(CD);
      Vector2D AD = new Vector2D(DA);
      BA.negate();
      CB.negate();
      DC.negate();
      AD.negate();

      FourBarVertex vertexA = fourBar.getVertexA();
      FourBarVertex vertexB = fourBar.getVertexB();
      FourBarVertex vertexC = fourBar.getVertexC();
      FourBarVertex vertexD = fourBar.getVertexD();

      assertEquals(DA.cross(AB) <= 0.0, vertexA.isConvex());
      assertEquals(AB.cross(BC) <= 0.0, vertexB.isConvex());
      assertEquals(BC.cross(CD) <= 0.0, vertexC.isConvex());
      assertEquals(CD.cross(DA) <= 0.0, vertexD.isConvex());

      double expectedAB = AB.length();
      double expectedBC = BC.length();
      double expectedCD = CD.length();
      double expectedDA = DA.length();
      double expectedDAB = AD.angle(AB);
      double expectedABC = BA.angle(BC);
      double expectedBCD = CB.angle(CD);
      double expectedCDA = DC.angle(DA);
      double expectedAC = A.distance(C);
      double expectedBD = B.distance(D);

      System.out.printf("Expected\n\tlengths: AB=%f, BC=%f, CD=%f, DA=%f\n\tangles: DAB=%f, ABC=%f, BCD=%f, CDA=%f\n\tdiagonal lengths: AC=%f, BD=%f\n",
                        expectedAB,
                        expectedBC,
                        expectedCD,
                        expectedDA,
                        expectedDAB,
                        expectedABC,
                        expectedBCD,
                        expectedCDA,
                        expectedAC,
                        expectedBD);

      try
      {
         assertTrue(expectedAC <= fourBar.getDiagonalAC().getMaxLength(),
                    "Inaccurate maxAC: valid length: " + expectedAC + ", computed max: " + fourBar.getDiagonalAC().getMaxLength());
         assertTrue(expectedBD <= fourBar.getDiagonalBD().getMaxLength(),
                    "Inaccurate maxBD: valid length: " + expectedBD + ", computed max: " + fourBar.getDiagonalBD().getMaxLength());

         assertTrue(vertexA.getMinAngle() <= expectedDAB,
                    "Itertation " + iteration + ", inaccurate minDAB: valid angle: " + expectedDAB + ", computed min: " + vertexA.getMinAngle());
         assertTrue(vertexB.getMinAngle() <= expectedABC,
                    "Itertation " + iteration + ", inaccurate minABC: valid angle: " + expectedABC + ", computed min: " + vertexB.getMinAngle());
         assertTrue(vertexC.getMinAngle() <= expectedBCD,
                    "Itertation " + iteration + ", inaccurate minBCD: valid angle: " + expectedBCD + ", computed min: " + vertexC.getMinAngle());
         assertTrue(vertexD.getMinAngle() <= expectedCDA,
                    "Itertation " + iteration + ", inaccurate minCDA: valid angle: " + expectedCDA + ", computed min: " + vertexD.getMinAngle());
         assertTrue(expectedDAB <= vertexA.getMaxAngle(),
                    "Itertation " + iteration + ", inaccurate maxDAB: valid angle: " + expectedDAB + ", computed max: " + vertexA.getMaxAngle());
         assertTrue(expectedABC <= vertexB.getMaxAngle(),
                    "Itertation " + iteration + ", inaccurate maxABC: valid angle: " + expectedABC + ", computed max: " + vertexB.getMaxAngle());
         assertTrue(expectedBCD <= vertexC.getMaxAngle(),
                    "Itertation " + iteration + ", inaccurate maxBCD: valid angle: " + expectedBCD + ", computed max: " + vertexC.getMaxAngle());
         assertTrue(expectedCDA <= vertexD.getMaxAngle(),
                    "Itertation " + iteration + ", inaccurate maxCDA: valid angle: " + expectedCDA + ", computed max: " + vertexD.getMaxAngle());

         switch (EuclidCoreRandomTools.nextElementIn(random, FourBarAngle.values()))
         {
            case DAB:
               fourBar.update(FourBarAngle.DAB, expectedDAB);
               break;
            case ABC:
               fourBar.update(FourBarAngle.ABC, expectedABC);
               break;
            case BCD:
               fourBar.update(FourBarAngle.BCD, expectedBCD);
               break;
            case CDA:
               fourBar.update(FourBarAngle.CDA, expectedCDA);
               break;
         }

         System.out.println(fourBar);

         assertEquals(expectedAC, fourBar.getDiagonalAC().getLength(), EPSILON, "Iteration " + iteration);
         assertEquals(expectedBD, fourBar.getDiagonalBD().getLength(), EPSILON, "Iteration " + iteration);

         assertEquals(expectedDAB, vertexA.getAngle(), EPSILON, "Iteration " + iteration + ", error: " + Math.abs(expectedDAB - vertexA.getAngle()));
         assertEquals(expectedABC, vertexB.getAngle(), EPSILON, "Iteration " + iteration + ", error: " + Math.abs(expectedABC - vertexB.getAngle()));
         assertEquals(expectedBCD, vertexC.getAngle(), EPSILON, "Iteration " + iteration + ", error: " + Math.abs(expectedBCD - vertexC.getAngle()));
         assertEquals(expectedCDA, vertexD.getAngle(), EPSILON, "Iteration " + iteration + ", error: " + Math.abs(expectedCDA - vertexD.getAngle()));
      }
      catch (Throwable e)
      {
         System.err.println(e.getClass().getSimpleName() + ": " + e.getMessage());

         Viewer viewer = startupViewer();
         viewer.updateFOV(A, B, C, D);
         draw(viewer, A, B, C, D);
         viewer.waitUntilClosed();
         throw e;
      }
   }

   private static class Viewer
   {
      double span = 10.0;
      Point2D center = new Point2D();

      Scene scene;
      Group root;
      final CountDownLatch countDownLatch = new CountDownLatch(1);

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
      private String strokeSyle;

      public VertexGraphics()
      {
      }

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
      PlatformImpl.runLater(() ->
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
            DoubleBinding xPositionProperty = xPositionProperty(viewer.span, viewer.center, scene, vertex);
            DoubleBinding yPositionProperty = yPositionProperty(viewer.span, viewer.center, scene, vertex);
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
            line.startXProperty().bind(xPositionProperty(viewer.span, viewer.center, scene, vertex));
            line.startYProperty().bind(yPositionProperty(viewer.span, viewer.center, scene, vertex));
            line.endXProperty().bind(xPositionProperty(viewer.span, viewer.center, scene, nextVertex));
            line.endYProperty().bind(yPositionProperty(viewer.span, viewer.center, scene, nextVertex));
            line.setStrokeWidth(2.0);
            line.setStroke(vertex.color.interpolate(nextVertex.color, 0.5));
            root.getChildren().add(line);
         }

         for (int i = 0; i < 4; i++)
         {
            VertexGraphics vertex = vertices[i];

            Circle circle = new Circle(5.0);
            circle.centerXProperty().bind(xPositionProperty(viewer.span, viewer.center, scene, vertex));
            circle.centerYProperty().bind(yPositionProperty(viewer.span, viewer.center, scene, vertex));
            circle.setFill(vertex.color);
            root.getChildren().add(circle);
         }
      });
   }

   public static DoubleBinding yPositionProperty(double span, Point2D center, Scene scene, Point2DReadOnly vertex)
   {
      return Bindings.multiply(0.5 - (vertex.getY() - center.getY()) / span, scene.heightProperty());
   }

   public static DoubleBinding xPositionProperty(double span, Point2D center, Scene scene, Point2DReadOnly vertex)
   {
      return Bindings.multiply(0.5 + (vertex.getX() - center.getX()) / span, scene.widthProperty());
   }
}
