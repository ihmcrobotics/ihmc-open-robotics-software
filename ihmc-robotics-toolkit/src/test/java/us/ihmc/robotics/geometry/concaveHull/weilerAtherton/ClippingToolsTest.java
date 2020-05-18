package us.ihmc.robotics.geometry.concaveHull.weilerAtherton;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.ClippingTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.LinkedPoint;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.LinkedPointList;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.WeilerAthertonPolygonClipping;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class ClippingToolsTest
{
   @Test
   public void testCreateLinkedPointList()
   {
      // check simple square
      ConcavePolygon2D polygon = new ConcavePolygon2D();
      polygon.addVertex(-1.0, 1.0);
      polygon.addVertex(1.0, 1.0);
      polygon.addVertex(1.0, -1.0);
      polygon.addVertex(-1.0, -1.0);
      polygon.update();

      LinkedPointList pointList = ClippingTools.createLinkedPointList(polygon);
      LinkedPoint point = pointList.getFirstPoint();

      // check the chain
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         point = point.getSuccessor();
      assertTrue(point == pointList.getFirstPoint());

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("Failed at vertex " + i, polygon.getVertex(i), point.getPoint(), 1e-6);
         point = point.getSuccessor();
      }

      // Check triangle
      polygon.clear();
      polygon.addVertex(0.0, 1.0);
      polygon.addVertex(1.0, 0.0);
      polygon.addVertex(-1.0, 0.0);
      polygon.update();

      pointList = ClippingTools.createLinkedPointList(polygon);
      point = pointList.getFirstPoint();

      // check the chain
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         point = point.getSuccessor();
      }
      assertTrue(point == pointList.getFirstPoint());

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("Failed at vertex " + i, polygon.getVertex(i), point.getPoint(), 1e-6);
         point = point.getSuccessor();
      }
   }

   @Test
   public void testAddingIntersections()
   {
      // Check two squares overlapping along edges
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.5, 0.5);
      polygonB.addVertex(1.5, 0.5);
      polygonB.addVertex(1.5, -0.5);
      polygonB.addVertex(0.5, -0.5);
      polygonB.update();

      LinkedPointList pointListA = ClippingTools.createLinkedPointList(polygonA);
      LinkedPointList pointListB = ClippingTools.createLinkedPointList(polygonB);

      ConcavePolygon2D polygonAWithIntersectionsExpected = new ConcavePolygon2D();
      polygonAWithIntersectionsExpected.addVertex(-1.0, 1.0);
      polygonAWithIntersectionsExpected.addVertex(1.0, 1.0);
      polygonAWithIntersectionsExpected.addVertex(1.0, 0.5);
      polygonAWithIntersectionsExpected.addVertex(1.0, -0.5);
      polygonAWithIntersectionsExpected.addVertex(1.0, -1.0);
      polygonAWithIntersectionsExpected.addVertex(-1.0, -1.0);
      polygonAWithIntersectionsExpected.update();


      ConcavePolygon2D polygonBWithIntersectionsExpected = new ConcavePolygon2D();
      polygonBWithIntersectionsExpected.addVertex(0.5, 0.5);
      polygonBWithIntersectionsExpected.addVertex(1.0, 0.5);
      polygonBWithIntersectionsExpected.addVertex(1.5, 0.5);
      polygonBWithIntersectionsExpected.addVertex(1.5, -0.5);
      polygonBWithIntersectionsExpected.addVertex(1.0, -0.5);
      polygonBWithIntersectionsExpected.addVertex(0.5, -0.5);
      polygonBWithIntersectionsExpected.update();

      ClippingTools.insertIntersectionsIntoList(pointListA, polygonB);

      LinkedPoint pointOnA = pointListA.getFirstPoint();
      for (int i = 0; i < polygonAWithIntersectionsExpected.getNumberOfVertices(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("Failed at vertex " + i, polygonAWithIntersectionsExpected.getVertex(i), pointOnA.getPoint(), 1e-6);
         pointOnA = pointOnA.getSuccessor();
      }
      assertTrue(pointOnA == pointListA.getFirstPoint());

      ClippingTools.insertIntersectionsIntoList(pointListB, polygonA);

      LinkedPoint pointOnB = pointListB.getFirstPoint();
      for (int i = 0; i < polygonAWithIntersectionsExpected.getNumberOfVertices(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("Failed at vertex " + i, polygonBWithIntersectionsExpected.getVertex(i), pointOnB.getPoint(), 1e-6);
         pointOnB = pointOnB.getSuccessor();
      }
      assertTrue(pointOnB == pointListB.getFirstPoint());
   }

   @Test
   public void testTrickyInsertIntersections()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(0.5, 0.5);
      clippingPolygon.update();

      LinkedPointList listA = ClippingTools.createLinkedPointList(polygonToClip);
      LinkedPointList listB = ClippingTools.createLinkedPointList(clippingPolygon);

      ClippingTools.insertIntersectionsIntoList(listA, clippingPolygon);
      ClippingTools.insertIntersectionsIntoList(listB, polygonToClip);

      LinkedPoint pointA = listA.getFirstPoint();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, 1.0), pointA.getPoint(), 1e-7);
      pointA = pointA.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 1.0), pointA.getPoint(), 1e-7);
      pointA = pointA.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), pointA.getPoint(), 1e-7);
      pointA = pointA.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -1.0), pointA.getPoint(), 1e-7);
      pointA = pointA.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, -1.0), pointA.getPoint(), 1e-7);

      LinkedPoint pointB = listB.getFirstPoint();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 1.5), pointB.getPoint(), 1e-7);
      pointB = pointB.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.5, 1.5), pointB.getPoint(), 1e-7);
      pointB = pointB.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), pointB.getPoint(), 1e-7);
      pointB = pointB.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 0.5), pointB.getPoint(), 1e-7);
      pointB = pointB.getSuccessor();
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 1.0), pointB.getPoint(), 1e-7);
   }

}
