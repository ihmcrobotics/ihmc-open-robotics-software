package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.Test;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class JTSToolsTest
{
   private static final int ITERATIONS = 1000;

   @Test(timeout = 30000)
   public void testPointConversions()
   {
      Random random = new Random(4234234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D expectedPoint2D = EuclidCoreRandomTools.nextPoint2D(random);
         Point3D expectedPoint3D = new Point3D(expectedPoint2D);
         Coordinate coordinate = JTSTools.point2DToCoordinate(expectedPoint2D);
         Point2D actualPoint2D = JTSTools.coordinateToPoint2D(coordinate);
         Point3D actualPoint3D = JTSTools.coordinateToPoint3D(coordinate);

         assertEquals(expectedPoint2D, actualPoint2D);
         assertEquals(expectedPoint3D, actualPoint3D);

         actualPoint2D = JTSTools.vertexToPoint2D(new Vertex(coordinate));
         actualPoint3D = JTSTools.vertexToPoint3D(new Vertex(coordinate));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D[] expectedPoint2Ds = random.ints(10).mapToObj(j -> EuclidCoreRandomTools.nextPoint2D(random)).toArray(Point2D[]::new);
         Coordinate[] coordinates = JTSTools.point2DsToCoordinates(expectedPoint2Ds);

         assertEquals(expectedPoint2Ds.length, coordinates.length);

         for (int j = 0; j < expectedPoint2Ds.length; j++)
         {
            Point2D actualPoint2D = JTSTools.coordinateToPoint2D(coordinates[j]);
            assertEquals(expectedPoint2Ds[j], actualPoint2D);
         }
      }
   }

   @Test(timeout = 30000)
   public void testLineStringConversion() throws Exception
   {
      Random random = new Random(2423423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment2D lineSegment2D = EuclidGeometryRandomTools.nextLineSegment2D(random);
         LineString lineString = JTSTools.lineSegment2DToLineString(lineSegment2D);

         assertEquals(lineSegment2D.getFirstEndpoint(), JTSTools.pointToPoint2D(lineString.getStartPoint()));
         assertEquals(lineSegment2D.getSecondEndpoint(), JTSTools.pointToPoint2D(lineString.getEndPoint()));
      }
   }

   @Test(timeout = 30000)
   public void testMultiString() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      { // Convert a close polygon to a MultiString
         ConvexPolygon2D convexPolygon2D = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 3.0, 20);

         List<LineSegment2D> edges = IntStream.range(0, convexPolygon2D.getNumberOfVertices())
                                              .mapToObj(j -> new LineSegment2D(convexPolygon2D.getVertex(j), convexPolygon2D.getNextVertex(j)))
                                              .collect(Collectors.toList());

         MultiLineString multiLineString = JTSTools.createMultiLineString(edges);
         assertEquals(1, multiLineString.getNumGeometries());
         LineString lineString = (LineString) multiLineString.getGeometryN(0);
         assertEquals(convexPolygon2D.getNumberOfVertices() + 1, lineString.getNumPoints());

         for (int j = 0; j <= convexPolygon2D.getNumberOfVertices(); j++)
         {
            Point2DReadOnly expectedPoint = convexPolygon2D.getVertex(j % convexPolygon2D.getNumberOfVertices());
            Point actualPoint = lineString.getPointN(j);
            assertEquals("Vertex: " + j, expectedPoint, JTSTools.pointToPoint2D(actualPoint));
         }

         // Now we flip the edges randomly
         edges.stream().filter(edge -> random.nextBoolean()).forEach(LineSegment2D::flipDirection);

         multiLineString = JTSTools.createMultiLineString(edges);
         assertEquals(1, multiLineString.getNumGeometries());
         lineString = (LineString) multiLineString.getGeometryN(0);
         assertEquals(convexPolygon2D.getNumberOfVertices() + 1, lineString.getNumPoints());

         for (int j = 0; j <= convexPolygon2D.getNumberOfVertices(); j++)
         {
            Point2DReadOnly expectedPoint = convexPolygon2D.getVertex(j % convexPolygon2D.getNumberOfVertices());
            Point actualPoint = lineString.getPointN(j);
            assertEquals("Vertex: " + j, expectedPoint, JTSTools.pointToPoint2D(actualPoint));
         }

         // Now we shuffle the edges
         Collections.shuffle(edges, random);
         multiLineString = JTSTools.createMultiLineString(edges);
         assertEquals("Problematic MultiLineString: " + multiLineString, 1, multiLineString.getNumGeometries());
         lineString = (LineString) multiLineString.getGeometryN(0);
         assertEquals(convexPolygon2D.getNumberOfVertices() + 1, lineString.getNumPoints());

         for (Coordinate coordinate : lineString.getCoordinates())
            assertTrue(convexPolygon2D.getVertexBufferView().contains(JTSTools.coordinateToPoint2D(coordinate)));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Convert two distinct and separated polygon to a MultiString
         ConvexPolygon2D polygon1 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 3.0, 20);
         ConvexPolygon2D polygon2 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 3.0, 30);

         // Translating polygon2 so the bounding boxes don't even touch
         Vector2D direction = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.1);
         while (polygon1.getBoundingBox().intersectsEpsilon(polygon2.getBoundingBox(), 0.01))
            polygon2.translate(direction);

         List<LineSegment2D> edges1 = IntStream.range(0, polygon1.getNumberOfVertices())
                                               .mapToObj(j -> new LineSegment2D(polygon1.getVertex(j), polygon1.getNextVertex(j))).collect(Collectors.toList());
         List<LineSegment2D> edges2 = IntStream.range(0, polygon2.getNumberOfVertices())
                                               .mapToObj(j -> new LineSegment2D(polygon2.getVertex(j), polygon2.getNextVertex(j))).collect(Collectors.toList());
         List<LineSegment2D> allEdges = new ArrayList<>();
         allEdges.addAll(edges1);
         allEdges.addAll(edges2);

         MultiLineString multiLineString = JTSTools.createMultiLineString(allEdges);
         assertEquals(2, multiLineString.getNumGeometries());
         LineString lineString1 = (LineString) multiLineString.getGeometryN(0);
         LineString lineString2 = (LineString) multiLineString.getGeometryN(1);

         assertEquals(polygon1.getNumberOfVertices() + 1, lineString1.getNumPoints());
         assertEquals(polygon2.getNumberOfVertices() + 1, lineString2.getNumPoints());

         for (int j = 0; j <= polygon1.getNumberOfVertices(); j++)
         {
            Point2DReadOnly expectedPoint = polygon1.getVertex(j % polygon1.getNumberOfVertices());
            Point actualPoint = lineString1.getPointN(j);
            assertEquals("Vertex: " + j, expectedPoint, JTSTools.pointToPoint2D(actualPoint));
         }

         for (int j = 0; j <= polygon2.getNumberOfVertices(); j++)
         {
            Point2DReadOnly expectedPoint = polygon2.getVertex(j % polygon2.getNumberOfVertices());
            Point actualPoint = lineString2.getPointN(j);
            assertEquals("Vertex: " + j, expectedPoint, JTSTools.pointToPoint2D(actualPoint));
         }
      }
   }
}
