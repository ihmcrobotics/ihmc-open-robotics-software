package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonShrinkerTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }


   @Test
   public void testSimpleSquareConvexPolygonShrinking()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(0.0, 0.0));
      vertices.add(new Point2D(1.0, 0.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(0.0, 1.0));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.1, 0.1), shrunkenPolygon.getVertexCCW(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.9, 0.1), shrunkenPolygon.getVertexCCW(1), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.9, 0.9), shrunkenPolygon.getVertexCCW(2), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.1, 0.9), shrunkenPolygon.getVertexCCW(3), 1e-7);


      polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));

      shrinker.scaleConvexPolygon(polygon, 1.1, shrunkenPolygon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, 0.5), shrunkenPolygon.getVertexCCW(0), 1e-7);
      assertEquals(1, shrunkenPolygon.getNumberOfVertices());
   }

   @Test
   public void testSimpleSquareConvexPolygonShrinkingWithIndexToIgnore()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(0.0, 0.0));
      vertices.add(new Point2D(1.0, 0.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(0.0, 1.0));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon, 1);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.1, 0.1), shrunkenPolygon.getVertexCCW(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 0.1), shrunkenPolygon.getVertexCCW(1), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 0.9), shrunkenPolygon.getVertexCCW(2), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.1, 0.9), shrunkenPolygon.getVertexCCW(3), 1e-7);


      polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));

      shrinker.scaleConvexPolygon(polygon, 1.1, shrunkenPolygon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, 0.5), shrunkenPolygon.getVertexCCW(0), 1e-7);
      assertEquals(1, shrunkenPolygon.getNumberOfVertices());
   }

   @Test
   public void testSimpleTriangleConvexPolygonShrinking()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(0.0, 0.5));
      vertices.add(new Point2D(1.0, 0.0));
      vertices.add(new Point2D(2.2, 1.0));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon);


      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.986224428207409, 0.11869118477128504), shrunkenPolygon.getVertexCCW(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.8160104213223893, 0.8101795123671021), shrunkenPolygon.getVertexCCW(1), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.2947361006115917, 0.4644353485691937), shrunkenPolygon.getVertexCCW(2), 1e-7);

      polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));

      shrinker.scaleConvexPolygon(polygon, 1.1, shrunkenPolygon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0666666666, 0.5), shrunkenPolygon.getVertexCCW(0), 1e-7);
      assertEquals(1, shrunkenPolygon.getNumberOfVertices());
   }


   @Test
   public void testSimpleLineConvexPolygonShrinking()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(-1.0, 3.0));
      vertices.add(new Point2D(1.0, 3.0));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.9, 3.0), shrunkenPolygon.getVertexCCW(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.9, 3.0), shrunkenPolygon.getVertexCCW(1), 1e-7);

      polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));

      shrinker.scaleConvexPolygon(polygon, 1.1, shrunkenPolygon);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 3.0), shrunkenPolygon.getVertexCCW(0), 1e-7);
      assertEquals(1, shrunkenPolygon.getNumberOfVertices());
   }

   @Test
   public void testSimplePointConvexPolygonShrinking()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(-1.0, 3.0));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, 3.0), shrunkenPolygon.getVertexCCW(0), 1e-7);
   }

   @Test
   public void testShrinkingRandomPolygonsAreCompletelyInsideOriginalPolygons()
   {
      Random random = new Random(1984L);
      ReferenceFrame zUpFrame = ReferenceFrame.getWorldFrame();

      double xMin = -2.0;
      double xMax = 2.0;
      double yMin = -1.0;
      double yMax = 4.0;
      double widthMax = 2.2;
      double heightMax = 1.3;
      int numberOfPoints = random.nextInt(20);
      int numberOfPolygons = 100;

      ArrayList<FrameConvexPolygon2D> randomPolygons = ConvexPolygon2dTestHelpers.generateRandomPolygons(random, zUpFrame, xMin, xMax, yMin, yMax, widthMax, heightMax, numberOfPoints, numberOfPolygons);

      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      FrameConvexPolygon2D shrunkenPolygon = new FrameConvexPolygon2D();

      for (FrameConvexPolygon2D randomPolygon : randomPolygons)
      {
         double distance = RandomNumbers.nextDouble(random, 0.001, 5.0);
         shrinker.scaleConvexPolygon(randomPolygon, distance, shrunkenPolygon);

         ConvexPolygon2DReadOnly bigPolygon = randomPolygon;
         ConvexPolygon2DReadOnly smallPolygon = shrunkenPolygon;

         boolean completelyInside = ConvexPolygon2dCalculator.isPolygonInside(smallPolygon, bigPolygon);
         assertTrue(completelyInside);
      }
   }

   // Use manually when making sure no garbage is generated or doing timing tests.
   @Disabled
   @Test
   public void testMemoryGarbageGeneration()
   {
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      vertices.add(new Point2D(0.0, 0.0));
      vertices.add(new Point2D(0.5, -0.2));
      vertices.add(new Point2D(1.0, 0.0));
      vertices.add(new Point2D(1.2, 0.5));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(0.5, 1.2));
      vertices.add(new Point2D(0.0, 1.0));
      vertices.add(new Point2D(-0.2, 0.5));

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

      int numberOfTests = 2000;

      long startTime = System.currentTimeMillis();
      for (int i=0; i<numberOfTests; i++)
      {
         shrinker.scaleConvexPolygon(polygon, 0.1, shrunkenPolygon);
      }
      long endTime = System.currentTimeMillis();

      double millisPerTest = ((double) (endTime - startTime)) / ((double) numberOfTests);

      System.out.println("millisPerTest = " + millisPerTest);
   }



}
