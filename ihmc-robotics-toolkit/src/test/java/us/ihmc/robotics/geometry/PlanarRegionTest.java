package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomGeometry;

public class PlanarRegionTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testIntersections()
   {
      List<ConvexPolygon2D> polygonsRegion1 = new ArrayList<>();

      ConvexPolygon2D polygon11 = new ConvexPolygon2D();
      polygon11.addVertex(0.0, 0.0);
      polygon11.addVertex(2.0, 0.0);
      polygon11.addVertex(2.0, 0.5);
      polygon11.addVertex(0.0, 0.5);
      polygon11.update();
      polygonsRegion1.add(polygon11);

      ConvexPolygon2D polygon12 = new ConvexPolygon2D();
      polygon12.addVertex(1.0, 0.0);
      polygon12.addVertex(2.0, 0.0);
      polygon12.addVertex(2.0, -1.5);
      polygon12.addVertex(1.0, -1.5);
      polygon12.update();
      polygonsRegion1.add(polygon12);

      RigidBodyTransform transform1 = new RigidBodyTransform();
      PlanarRegion region1 = new PlanarRegion(transform1, polygonsRegion1);

      List<ConvexPolygon2D> polygonsRegion2 = new ArrayList<>();
      ConvexPolygon2D polygon21 = new ConvexPolygon2D();
      polygon21.addVertex(-1.0, 0.1);
      polygon21.addVertex(1.0, 0.1);
      polygon21.addVertex(1.0, -0.1);
      polygon21.addVertex(-1.0, -0.1);
      polygon21.update();
      polygonsRegion2.add(polygon21);

      ConvexPolygon2D polygon22 = new ConvexPolygon2D();
      polygon22.addVertex(1.5, 0.1);
      polygon22.addVertex(2.0, 0.1);
      polygon22.addVertex(2.0, -0.1);
      polygon22.addVertex(1.5, -0.1);
      polygon22.update();
      polygonsRegion2.add(polygon22);

      RigidBodyTransform transform2 = new RigidBodyTransform();
      transform2.setTranslation(0.5, 0.0, 0.0);
      transform2.appendYawRotation(-Math.PI / 4.0);
      transform2.appendRollRotation(Math.PI / 2.0);
      PlanarRegion region2 = new PlanarRegion(transform2, polygonsRegion2);

      List<LineSegment3D> intersections = region1.intersect(region2);

      List<LineSegment3D> expectedIntersections = new ArrayList<>();
      expectedIntersections.add(new LineSegment3D(new Point3D(0.0, 0.5, 0.0), new Point3D(0.5, 0.0, 0.0)));
      expectedIntersections.add(new LineSegment3D(new Point3D(1.0, -0.5, 0.0), new Point3D(0.5 + 1.0 / Math.sqrt(2.0), -1.0 / Math.sqrt(2.0), 0.0)));
      expectedIntersections.add(new LineSegment3D(new Point3D(0.5 + 1.5 / Math.sqrt(2.0), -1.5 / Math.sqrt(2.0), 0.0),
                                                  new Point3D(0.5 + 2.0 / Math.sqrt(2.0), -2.0 / Math.sqrt(2.0), 0.0)));

      Assert.assertEquals(expectedIntersections.size(), intersections.size());
      for (LineSegment3D intersection : intersections)
      {
         boolean foundMatch = false;
         for (LineSegment3D expectedIntersection : expectedIntersections)
         {
            if (intersection.getDirection(false).dot(expectedIntersection.getDirection(false)) < 0.0)
            {
               expectedIntersection.flipDirection();
            }
            if (intersection.epsilonEquals(expectedIntersection, 1.0e-10))
            {
               foundMatch = true;
            }
         }
         Assert.assertTrue(foundMatch);
      }
   }

   @Test
   public void testIsPointInWorld2DInside()
   {
      Random random = new Random(1776L);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Point3D regionTranslation = new Point3D();
      Point3D pointAbove = new Point3D();
      Point3D pointBelow = new Point3D();
      Vector3D regionNormal = new Vector3D();

      for (int i = 0; i < 10000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, 1, 10.0, 5);
         planarRegion.getTransformToWorld(transformToWorld);

         Point2DReadOnly centroid = planarRegion.getLastConvexPolygon().getCentroid();
         regionTranslation.set(centroid.getX(), centroid.getY(), 0.0);
         transformToWorld.transform(regionTranslation);
         regionTranslation.setZ(planarRegion.getPlaneZGivenXY(regionTranslation.getX(), regionTranslation.getY()));

         planarRegion.getNormal(regionNormal);

         regionNormal.normalize();
         regionNormal.scale(1e-6);

         pointAbove.add(regionTranslation, regionNormal);
         pointBelow.sub(regionTranslation, regionNormal);

         assertTrue(planarRegion.isPointInWorld2DInside(pointAbove));
         assertTrue(planarRegion.isPointInWorld2DInside(pointBelow));
      }
   }

   @Test
   public void testIsPointOn()
   {
      Random random = new Random(1776L);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Point3D regionTranslation = new Point3D();
      Point3D pointAbove = new Point3D();
      Point3D pointBelow = new Point3D();
      Vector3D regionNormal = new Vector3D();

      for (int i = 0; i < 10000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, 1, 10.0, 5);
         planarRegion.getTransformToWorld(transformToWorld);

         Point2DReadOnly centroid = planarRegion.getLastConvexPolygon().getCentroid();
         regionTranslation.set(centroid.getX(), centroid.getY(), 0.0);
         transformToWorld.transform(regionTranslation);
         regionTranslation.setZ(planarRegion.getPlaneZGivenXY(regionTranslation.getX(), regionTranslation.getY()));

         planarRegion.getNormal(regionNormal);

         regionNormal.normalize();
         regionNormal.scale(1e-6);

         pointAbove.add(regionTranslation, regionNormal);
         pointBelow.sub(regionTranslation, regionNormal);

         assertTrue(planarRegion.isPointOnOrSlightlyAbove(pointAbove, 1e-5));
         assertFalse(planarRegion.isPointOnOrSlightlyAbove(pointBelow, 1e-5));
      }
   }

   @Test
   public void testIsPointOnOrSlightlyBelow()
   {
      Random random = new Random(1776L);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Point3D regionTranslation = new Point3D();
      Point3D pointAbove = new Point3D();
      Point3D pointBelow = new Point3D();
      Vector3D regionNormal = new Vector3D();

      for (int i = 0; i < 10000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, 1, 10.0, 5);
         planarRegion.getTransformToWorld(transformToWorld);

         Point2DReadOnly centroid = planarRegion.getLastConvexPolygon().getCentroid();
         regionTranslation.set(centroid.getX(), centroid.getY(), 0.0);
         transformToWorld.transform(regionTranslation);
         regionTranslation.setZ(planarRegion.getPlaneZGivenXY(regionTranslation.getX(), regionTranslation.getY()));

         planarRegion.getNormal(regionNormal);

         regionNormal.normalize();
         regionNormal.scale(1e-6);

         pointAbove.add(regionTranslation, regionNormal);
         pointBelow.sub(regionTranslation, regionNormal);

         assertTrue(planarRegion.isPointOnOrSlightlyBelow(pointBelow, 1e-5));
         assertFalse(planarRegion.isPointOnOrSlightlyBelow(pointAbove, 1e-5));
      }
   }

   @Test
   public void testCreationOfBoundingBoxWithAllPointsGreaterThanOrigin()
   {
      final double zLocationOfPlanarRegion = 2.0;
      Point3D minPoint = new Point3D(1.0, 1.0, zLocationOfPlanarRegion);
      Point3D maxPoint = new Point3D(2.0, 2.0, zLocationOfPlanarRegion);

      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(minPoint.getX(), minPoint.getY());
      polygon1.addVertex(maxPoint.getX(), minPoint.getY());
      polygon1.addVertex(minPoint.getX(), maxPoint.getY());
      polygon1.addVertex(maxPoint.getX(), maxPoint.getY());

      regionConvexPolygons.add(polygon1);

      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      regionTransform.appendTranslation(0.0, 0.0, zLocationOfPlanarRegion);
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      assertThatAllPolygonVerticesAreInBoundingBox(regionConvexPolygons, planarRegion, boundingBox3dInWorld);

      Point3D boundingBoxMinPoint = new Point3D();
      Point3D boundingBoxMaxPoint = new Point3D();

      boundingBox3dInWorld.getMinPoint(boundingBoxMinPoint);
      boundingBox3dInWorld.getMaxPoint(boundingBoxMaxPoint);

      assertEquals(minPoint, boundingBoxMinPoint);
      assertEquals(maxPoint, boundingBoxMaxPoint);
   }

   @Test
   public void testCreationOfBoundingBoxWithAllPointsLessThanOrigin()
   {
      final double zLocationOfPlanarRegion = -2.0;
      Point3D maxPoint = new Point3D(-1.0, -1.0, zLocationOfPlanarRegion);
      Point3D minPoint = new Point3D(-2.0, -2.0, zLocationOfPlanarRegion);

      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(minPoint.getX(), minPoint.getY());
      polygon1.addVertex(maxPoint.getX(), minPoint.getY());
      polygon1.addVertex(minPoint.getX(), maxPoint.getY());
      polygon1.addVertex(maxPoint.getX(), maxPoint.getY());

      regionConvexPolygons.add(polygon1);

      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      regionTransform.appendTranslation(0.0, 0.0, zLocationOfPlanarRegion);
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      assertThatAllPolygonVerticesAreInBoundingBox(regionConvexPolygons, planarRegion, boundingBox3dInWorld);

      Point3D boundingBoxMinPoint = new Point3D();
      Point3D boundingBoxMaxPoint = new Point3D();

      boundingBox3dInWorld.getMinPoint(boundingBoxMinPoint);
      boundingBox3dInWorld.getMaxPoint(boundingBoxMaxPoint);

      assertEquals(minPoint, boundingBoxMinPoint);
      assertEquals(maxPoint, boundingBoxMaxPoint);
   }

   @Test
   public void testCreationOfBoundingBoxWithMinimumLessThanOriginAndMaximumGreaterThanOrigin()
   {
      Point3D maxPoint = new Point3D(2.0, 2.0, 0.0);
      Point3D minPoint = new Point3D(-2.0, -2.0, 0.0);

      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(minPoint.getX(), minPoint.getY());
      polygon1.addVertex(maxPoint.getX(), minPoint.getY());
      polygon1.addVertex(minPoint.getX(), maxPoint.getY());
      polygon1.addVertex(maxPoint.getX(), maxPoint.getY());

      regionConvexPolygons.add(polygon1);

      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      assertThatAllPolygonVerticesAreInBoundingBox(regionConvexPolygons, planarRegion, boundingBox3dInWorld);

      Point3D boundingBoxMinPoint = new Point3D();
      Point3D boundingBoxMaxPoint = new Point3D();

      boundingBox3dInWorld.getMinPoint(boundingBoxMinPoint);
      boundingBox3dInWorld.getMaxPoint(boundingBoxMaxPoint);

      assertEquals(minPoint, boundingBoxMinPoint);
      assertEquals(maxPoint, boundingBoxMaxPoint);
   }

   @Test
   public void testBoundingBoxForLShapedPlanarRegionWithIdentifyTransform()
   {
      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(-1.0, 1.0);
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(3.0, 1.0);
      polygon2.addVertex(3.0, -1.0);
      polygon2.addVertex(1.0, -1.0);
      polygon2.addVertex(1.0, 1.0);
      ConvexPolygon2D polygon3 = new ConvexPolygon2D();
      polygon3.addVertex(1.0, 3.0);
      polygon3.addVertex(1.0, 1.0);
      polygon3.addVertex(-1.0, 1.0);
      polygon3.addVertex(-1.0, 3.0);

      regionConvexPolygons.add(polygon1);
      regionConvexPolygons.add(polygon2);
      regionConvexPolygons.add(polygon3);
      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      assertThatAllPolygonVerticesAreInBoundingBox(regionConvexPolygons, planarRegion, boundingBox3dInWorld);
   }

   @Test
   public void testWithLShapedPlanarRegionWithIdentityTransform()
   {
      // polygons forming a L-shaped region.
      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(-1.0, 1.0);
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(3.0, 1.0);
      polygon2.addVertex(3.0, -1.0);
      polygon2.addVertex(1.0, -1.0);
      polygon2.addVertex(1.0, 1.0);
      ConvexPolygon2D polygon3 = new ConvexPolygon2D();
      polygon3.addVertex(1.0, 3.0);
      polygon3.addVertex(1.0, 1.0);
      polygon3.addVertex(-1.0, 1.0);
      polygon3.addVertex(-1.0, 3.0);

      regionConvexPolygons.add(polygon1);
      regionConvexPolygons.add(polygon2);
      regionConvexPolygons.add(polygon3);
      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      assertEquals("Wrong number of convex polygons in the region.", 3, planarRegion.getNumberOfConvexPolygons());
      for (int i = 0; i < 3; i++)
         assertTrue("Unexpected region polygon.", regionConvexPolygons.get(i).epsilonEquals(planarRegion.getConvexPolygon(i), 1.0e-10));

      Vector3D actualNormal = new Vector3D();
      planarRegion.getNormal(actualNormal);
      EuclidCoreTestTools.assertTuple3DEquals("Wrong region normal.", new Vector3D(0.0, 0.0, 1.0), actualNormal, 1.0e-10);
      Point3D actualOrigin = new Point3D();
      planarRegion.getPointInRegion(actualOrigin);
      EuclidCoreTestTools.assertTuple3DEquals("Wrong region origin.", new Point3D(), actualOrigin, 1.0e-10);
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(actualTransform);
      assertTrue("Wrong region transform to world.", regionTransform.epsilonEquals(actualTransform, 1.0e-10));

      Point2D point2d = new Point2D();

      // Do a bunch of trivial queries with isPointInside(Point2d) method.
      point2d.set(0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(2.0, 0.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(0.0, 2.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(2.0, 2.0);
      assertFalse(planarRegion.isPointInside(point2d));

      Point3D point3d = new Point3D();
      double maximumOrthogonalDistance = 1.0e-3;
      // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point in plane
      point3d.set(0.0, 0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 2.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 2.0, 0.0);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point below plane
      point3d.set(0.0, 0.0, -0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 0.0, -0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 2.0, -0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 0.0, -1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 0.0, -1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 2.0, -1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point above plane
      point3d.set(0.0, 0.0, 0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 0.0, 0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 2.0, 0.5 * maximumOrthogonalDistance);
      assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 0.0, 1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(2.0, 0.0, 1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
      point3d.set(0.0, 2.0, 1.5 * maximumOrthogonalDistance);
      assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));

      // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(double, double) method.
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(0.0, 0.0));
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(2.0, 0.0));
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(0.0, 2.0));
      assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(2.0, 2.0));

      // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point2d) method.
      point2d.set(0.0, 0.0);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
      point2d.set(2.0, 0.0);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
      point2d.set(0.0, 2.0);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
      point2d.set(2.0, 2.0);
      assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));

      // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point3D) method.
      point3d.set(0.0, 0.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(2.0, 0.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(0.0, 2.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(2.0, 2.0, Double.POSITIVE_INFINITY);
      assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));

      // Do a bunch of trivial queries with isLineSegmentIntersecting(LineSegment2d) method.
      LineSegment2D lineSegment = new LineSegment2D(0.0, 0.0, 2.0, 2.0);
      assertTrue(planarRegion.isLineSegmentIntersecting(lineSegment));
      List<Point2DBasics[]> intersectionsInPlaneFrame = new ArrayList<>();
      planarRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegment, intersectionsInPlaneFrame);
      assertEquals(3, intersectionsInPlaneFrame.size());

      lineSegment = new LineSegment2D(0.0, 0.0, 0.5, 0.5);
      assertFalse("Not intersecting if fully inside a single polygon", planarRegion.isLineSegmentIntersecting(lineSegment));
      intersectionsInPlaneFrame.clear();
      planarRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegment, intersectionsInPlaneFrame);
      assertEquals(0, intersectionsInPlaneFrame.size());

      lineSegment = new LineSegment2D(0.0, 0.0, 0.0, 1.5);
      assertTrue("Intersecting if fully inside but cross two polygons", planarRegion.isLineSegmentIntersecting(lineSegment));
      intersectionsInPlaneFrame.clear();
      planarRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegment, intersectionsInPlaneFrame);
      assertEquals(2, intersectionsInPlaneFrame.size());
      Point2DBasics[] points = intersectionsInPlaneFrame.get(0);
      assertEquals(1, points.length);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 1.0), points[0], 1e-7);
      points = intersectionsInPlaneFrame.get(1);
      assertEquals(1, points.length);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 1.0), points[0], 1e-7);

      lineSegment = new LineSegment2D(2.5, 0.5, 3.0, 9.0);
      assertTrue(planarRegion.isLineSegmentIntersecting(lineSegment));
      lineSegment = new LineSegment2D(2.5, 4.5, 3.0, 9.0);
      assertFalse("Not intersecting if fully outside", planarRegion.isLineSegmentIntersecting(lineSegment));

      lineSegment = new LineSegment2D(2.0, -2.0, 2.0, 2.0);
      assertTrue(planarRegion.isLineSegmentIntersecting(lineSegment));
      intersectionsInPlaneFrame.clear();
      planarRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegment, intersectionsInPlaneFrame);
      assertEquals(1, intersectionsInPlaneFrame.size());
      points = intersectionsInPlaneFrame.get(0);
      assertEquals(2, points.length);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, 1.0), points[0], 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, -1.0), points[1], 1e-7);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      // Do a bunch of trivial queries with isPolygonIntersecting(ConvexPolygon2d) method.
      assertTrue(planarRegion.isPolygonIntersecting(convexPolygon));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(2.0, 0.0, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(0.0, 2.0, convexPolygon)));
      assertFalse(planarRegion.isPolygonIntersecting(translateConvexPolygon(2.0, 2.0, convexPolygon)));
      assertFalse(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.21, 1.21, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.09, 1.09, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.21, 1.09, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.09, 1.21, convexPolygon)));

      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(convexPolygon, intersections);
      assertEquals(1, intersections.size());

      intersections.clear();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(translateConvexPolygon(2.0, 0.0, convexPolygon), intersections);
      assertEquals(1, intersections.size());

      intersections.clear();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(translateConvexPolygon(0.0, 2.0, convexPolygon), intersections);
      assertEquals(1, intersections.size());

      intersections.clear();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(translateConvexPolygon(2.0, 2.0, convexPolygon), intersections);
      assertEquals(0, intersections.size());

      intersections.clear();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(translateConvexPolygon(1.21, 1.21, convexPolygon), intersections);
      assertEquals(0, intersections.size());

      intersections.clear();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(translateConvexPolygon(1.09, 1.09, convexPolygon), intersections);
      assertEquals(3, intersections.size());
   }

   @Test
   public void testGetPolygonIntersectionsWhenSnapped()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(0.1, 0.2, 0.3);
      transform.setTranslation(1.2, 3.4, 5.6);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      PlanarRegion planarRegion = new PlanarRegion(transform, convexPolygon);

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
      polygonToSnap.addVertex(0.1, 0.1);
      polygonToSnap.addVertex(0.1, -0.1);
      polygonToSnap.addVertex(-0.1, -0.1);
      polygonToSnap.addVertex(-0.1, 0.1);
      polygonToSnap.update();

      RigidBodyTransform snappingTransform = new RigidBodyTransform();
      snappingTransform.setRotationEulerAndZeroTranslation(0.1, 0.2, 0.3);
      snappingTransform.setTranslation(1.2, 3.4, 5.6);

      double intersectionArea = planarRegion.getPolygonIntersectionAreaWhenSnapped(polygonToSnap, snappingTransform);
      assertEquals(0.04, intersectionArea, 1e-7);
   }

   @Test
   public void testWithLShapedPlanarRegionWithRandomTransform()
   {
      Random random = new Random(42L);

      // polygons forming a L-shaped region.
      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(-1.0, 1.0);
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(3.0, 1.0);
      polygon2.addVertex(3.0, -1.0);
      polygon2.addVertex(1.0, -1.0);
      polygon2.addVertex(1.0, 1.0);
      ConvexPolygon2D polygon3 = new ConvexPolygon2D();
      polygon3.addVertex(1.0, 3.0);
      polygon3.addVertex(1.0, 1.0);
      polygon3.addVertex(-1.0, 1.0);
      polygon3.addVertex(-1.0, 3.0);

      regionConvexPolygons.add(polygon1);
      regionConvexPolygons.add(polygon2);
      regionConvexPolygons.add(polygon3);
      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int iteration = 0; iteration < 10; iteration++)
      {
         Quaternion orientation = RandomGeometry.nextQuaternion(random, Math.toRadians(45.0));
         Vector3D translation = RandomGeometry.nextVector3D(random, 10.0);
         RigidBodyTransform regionTransform = new RigidBodyTransform(orientation, translation);
         ReferenceFrame localFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("local", worldFrame, regionTransform);
         PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

         assertEquals("Wrong number of convex polygons in the region.", 3, planarRegion.getNumberOfConvexPolygons());
         for (int i = 0; i < 3; i++)
            assertTrue("Unexpected region polygon.", regionConvexPolygons.get(i).epsilonEquals(planarRegion.getConvexPolygon(i), 1.0e-10));

         Vector3D expectedNormal = new Vector3D(0.0, 0.0, 1.0);
         regionTransform.transform(expectedNormal);
         Vector3D actualNormal = new Vector3D();
         planarRegion.getNormal(actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals("Wrong region normal.", expectedNormal, actualNormal, 1.0e-10);
         Point3D expectedOrigin = new Point3D();
         regionTransform.transform(expectedOrigin);
         Point3D actualOrigin = new Point3D();
         planarRegion.getPointInRegion(actualOrigin);
         EuclidCoreTestTools.assertTuple3DEquals("Wrong region origin.", expectedOrigin, actualOrigin, 1.0e-10);
         RigidBodyTransform actualTransform = new RigidBodyTransform();
         planarRegion.getTransformToWorld(actualTransform);
         assertTrue("Wrong region transform to world.", regionTransform.epsilonEquals(actualTransform, 1.0e-10));

         FramePoint2D point2d = new FramePoint2D();

         // Do a bunch of trivial queries with isPointInside(Point2d) method.
         point2d.setIncludingFrame(localFrame, 0.0, 0.0);
         assertTrue(planarRegion.isPointInside(point2d));
         point2d.setIncludingFrame(localFrame, 2.0, 0.0);
         assertTrue(planarRegion.isPointInside(point2d));
         point2d.setIncludingFrame(localFrame, 0.0, 2.0);
         assertTrue(planarRegion.isPointInside(point2d));
         point2d.setIncludingFrame(localFrame, 2.0, 2.0);
         assertFalse(planarRegion.isPointInside(point2d));

         FramePoint3D point3d = new FramePoint3D();
         double maximumOrthogonalDistance = 1.0e-3;
         // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point in plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point below plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, -0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, -0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, -0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, -1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, -1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, -1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         // Do a bunch of trivial queries with isPointInside(Point3D, double) method. Point above plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 1.5 * maximumOrthogonalDistance);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d, maximumOrthogonalDistance));

         // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(double, double) method.
         point2d.setIncludingFrame(localFrame, 0.0, 0.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY()));
         point2d.setIncludingFrame(localFrame, 2.0, 0.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY()));
         point2d.setIncludingFrame(localFrame, 0.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY()));
         point2d.setIncludingFrame(localFrame, 2.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY()));

         // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point2d) method.
         point2d.setIncludingFrame(localFrame, 0.0, 0.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
         point2d.setIncludingFrame(localFrame, 2.0, 0.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
         point2d.setIncludingFrame(localFrame, 0.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));
         point2d.setIncludingFrame(localFrame, 2.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d));

         // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point3D) method.
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
         point3d.setIncludingFrame(localFrame, 2.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));

         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
         convexPolygon.addVertex(0.2, 0.2);
         convexPolygon.addVertex(0.2, -0.2);
         convexPolygon.addVertex(-0.2, -0.2);
         convexPolygon.addVertex(-0.2, 0.2);
         convexPolygon.update();

         // Do a bunch of trivial queries with isPolygonIntersecting(ConvexPolygon2d) method.
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, convexPolygon)));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(2.0, 0.0, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(0.0, 2.0, convexPolygon))));
         assertFalse(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(2.0, 2.0, convexPolygon))));
         assertFalse(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.21, 1.21, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.09, 1.09, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.21, 1.09, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.09, 1.21, convexPolygon))));

         BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transformToWorld);

         for (ConvexPolygon2D convexPolygon2d : regionConvexPolygons)
         {
            ConvexPolygon2D convexPolygon2dInWorld = new ConvexPolygon2D(convexPolygon2d);
            convexPolygon2dInWorld.applyTransform(transformToWorld, false);
            for (int i = 0; i < convexPolygon2dInWorld.getNumberOfVertices(); i++)
            {
               Point2DReadOnly vertex = convexPolygon2dInWorld.getVertex(i);
               double planeZGivenXY = planarRegion.getPlaneZGivenXY(vertex.getX(), vertex.getY());

               assertTrue(
                     "Polygon vertex is not inside computed bounding box.\nVertex: " + vertex + "\nPlane z at vertex: " + planeZGivenXY + "\nBounding Box: "
                           + boundingBox3dInWorld, boundingBox3dInWorld.isInsideEpsilon(vertex.getX(), vertex.getY(), planeZGivenXY, 1e-15));
            }
         }
      }
   }

   @Test
   public void testGetPlaneZGivenXY()
   {
      ConvexPolygon2D convexPolygon2d = new ConvexPolygon2D();
      convexPolygon2d.addVertex(1.0, 1.0);
      convexPolygon2d.addVertex(-1.0, 1.0);
      convexPolygon2d.addVertex(-1.0, -1.0);
      convexPolygon2d.addVertex(1.0, -1.0);
      convexPolygon2d.update();
      ArrayList<ConvexPolygon2D> polygonList = new ArrayList<>();
      polygonList.add(convexPolygon2d);
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, polygonList);

      double xWorld = 0.0;
      double yWorld = 0.0;
      double planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      assertEquals(0.0, planeZGivenXY, 1e-7);

      transformToWorld.setTranslation(1.0, 2.0, 3.0);
      planarRegion = new PlanarRegion(transformToWorld, polygonList);
      planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      assertEquals(3.0, planeZGivenXY, 1e-7);

      double angle = Math.PI / 4.0;
      transformToWorld.setRotationEulerAndZeroTranslation(0.0, angle, 0.0);
      planarRegion = new PlanarRegion(transformToWorld, polygonList);
      xWorld = 1.3;
      planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      assertEquals(-xWorld * Math.tan(angle), planeZGivenXY, 1e-7);
      assertTrue(planarRegion.isPointInside(new Point3D(0.0, 0.0, 0.0), 1e-7));

      // Try really close to 90 degrees
      angle = Math.PI / 2.0 - 0.001;
      transformToWorld.setRotationEulerAndZeroTranslation(0.0, angle, 0.0);
      planarRegion = new PlanarRegion(transformToWorld, polygonList);
      xWorld = 1.3;
      planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      assertEquals(-xWorld * Math.tan(angle), planeZGivenXY, 1e-7);
      assertTrue(planarRegion.isPointInside(new Point3D(0.0, 0.0, 0.0), 1e-7));

      // Exactly 90 degrees.
      angle = Math.PI / 2.0 - 0.1;
      transformToWorld.setRotationEulerAndZeroTranslation(0.0, angle, 0.0);
      planarRegion = new PlanarRegion(transformToWorld, polygonList);
      xWorld = 1.3;
      planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      // With numerical roundoff, Math.tan(Math.PI/2.0) is not NaN:
      double tangent = Math.tan(angle);
      boolean planeZGivenXYIsNaN = Double.isNaN(planeZGivenXY);
      boolean valueMatchesComputed = (Math.abs(planeZGivenXY - -tangent * xWorld) < 1e-7);
      assertFalse(planeZGivenXYIsNaN);
      assertTrue(valueMatchesComputed);
      assertTrue(planarRegion.isPointInside(new Point3D(0.0, 0.0, 0.0), 1e-7));

      // If we set the transform to exactly z axis having no zWorld component (exactly 90 degree rotation about y in this case), then we do get NaN.
      // However (0, 0, 0) should still be inside. As should (0, 0, 0.5)
      transformToWorld.set(new double[] { 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });
      planarRegion = new PlanarRegion(transformToWorld, polygonList);
      xWorld = 1.3;
      planeZGivenXY = planarRegion.getPlaneZGivenXY(xWorld, yWorld);

      planeZGivenXYIsNaN = Double.isNaN(planeZGivenXY);
      valueMatchesComputed = (Math.abs(planeZGivenXY - -tangent * xWorld) < 1e-7);
      assertTrue(planeZGivenXYIsNaN);
      assertFalse(valueMatchesComputed);
      assertTrue(planarRegion.isPointInside(new Point3D(0.0, 0.0, 0.0), 1e-7));
      assertTrue(planarRegion.isPointInside(new Point3D(0.0, 0.0, 0.5), 1e-7));
   }

   @Test
   public void testGetSupportingVertex()
   {
      Random random = new Random(3290);

      // test simple unit square
      ConvexPolygon2D convexPolygon2d = new ConvexPolygon2D();
      convexPolygon2d.addVertex(1.0, 1.0);
      convexPolygon2d.addVertex(-1.0, 1.0);
      convexPolygon2d.addVertex(-1.0, -1.0);
      convexPolygon2d.addVertex(1.0, -1.0);
      convexPolygon2d.update();
      ArrayList<ConvexPolygon2D> polygonList = new ArrayList<>();
      polygonList.add(convexPolygon2d);
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, polygonList);

      assertTrue(planarRegion.getSupportingVertex(new Vector3D(1.0, 1.0, 0.0)).epsilonEquals(new Point3D(1.0, 1.0, 0.0), 1e-10));
      assertTrue(planarRegion.getSupportingVertex(new Vector3D(-1.0, 1.0, 0.0)).epsilonEquals(new Point3D(-1.0, 1.0, 0.0), 1e-10));
      assertTrue(planarRegion.getSupportingVertex(new Vector3D(-1.0, -1.0, 0.0)).epsilonEquals(new Point3D(-1.0, -1.0, 0.0), 1e-10));
      assertTrue(planarRegion.getSupportingVertex(new Vector3D(1.0, -1.0, 0.0)).epsilonEquals(new Point3D(1.0, -1.0, 0.0), 1e-10));

      // test random regions
      for (int i = 0; i < 10000; i++)
      {
         planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, 1, 5.0, 8);
         planarRegion.getTransformToWorld(transformToWorld);

         List<Point3D> convexHullVertices = planarRegion.getConvexHull().getPolygonVerticesView().stream().map(Point3D::new).peek(transformToWorld::transform).collect(Collectors.toList());
         Point3DReadOnly expectedSupportVertex, actualSupportVertex;
         Vector3D supportDirection = new Vector3D();

         // Trivial case #1: supportingVector = +X
         supportDirection.set(Axis.X);
         expectedSupportVertex = convexHullVertices.stream().max(Comparator.comparingDouble(Point3D::getX)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Trivial case #2: supportingVector = -X
         supportDirection.setAndNegate(Axis.X);
         expectedSupportVertex = convexHullVertices.stream().min(Comparator.comparingDouble(Point3D::getX)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Trivial case #1: supportingVector = +Y
         supportDirection.set(Axis.Y);
         expectedSupportVertex = convexHullVertices.stream().max(Comparator.comparingDouble(Point3D::getY)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Trivial case #2: supportingVector = -Y
         supportDirection.setAndNegate(Axis.Y);
         expectedSupportVertex = convexHullVertices.stream().min(Comparator.comparingDouble(Point3D::getY)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Trivial case #1: supportingVector = +Z
         supportDirection.set(Axis.Z);
         expectedSupportVertex = convexHullVertices.stream().max(Comparator.comparingDouble(Point3D::getZ)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Trivial case #2: supportingVector = -Z
         supportDirection.setAndNegate(Axis.Z);
         expectedSupportVertex = convexHullVertices.stream().min(Comparator.comparingDouble(Point3D::getZ)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));

         // Random support direction
         supportDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D orthogonalDirection = new Vector3D();
         Vector3D supportDirectionInPlane = new Vector3D();
         orthogonalDirection.cross(planarRegion.getNormal(), supportDirection);
         supportDirectionInPlane.cross(orthogonalDirection, planarRegion.getNormal());
         supportDirectionInPlane.normalize();
         supportDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));

         Line3D line = new Line3D();
         line.setDirection(orthogonalDirection);
         line.translate(20.0 * supportDirectionInPlane.getX(), 20.0 * supportDirectionInPlane.getY(), 20.0 * supportDirectionInPlane.getZ());

         expectedSupportVertex = convexHullVertices.stream().min(Comparator.comparingDouble(line::distance)).get();
         actualSupportVertex = planarRegion.getSupportingVertex(supportDirection);
         assertTrue("iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex, expectedSupportVertex.equals(actualSupportVertex));
      }
   }

   static ConvexPolygon2DBasics translateConvexPolygon(double xTranslation, double yTranslation, ConvexPolygon2DReadOnly convexPolygon)
   {
      Vector2D translation = new Vector2D(xTranslation, yTranslation);
      return convexPolygon.translateCopy(translation);
   }

   private static ConvexPolygon2D transformConvexPolygon(RigidBodyTransform transform, ConvexPolygon2DReadOnly convexPolygon2D)
   {
      ConvexPolygon2D transformedConvexPolygon = new ConvexPolygon2D(convexPolygon2D);
      transformedConvexPolygon.applyTransform(transform, false);
      return transformedConvexPolygon;
   }

   private void assertThatAllPolygonVerticesAreInBoundingBox(List<ConvexPolygon2D> regionConvexPolygons, PlanarRegion planarRegion,
         BoundingBox3D boundingBox3dInWorld)
   {
      for (ConvexPolygon2D convexPolygon2dInWorld : regionConvexPolygons)
      {
         for (int i = 0; i < convexPolygon2dInWorld.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = convexPolygon2dInWorld.getVertex(i);
            double planeZGivenXY = planarRegion.getPlaneZGivenXY(vertex.getX(), vertex.getY());

            assertTrue("Polygon vertex is not inside computed bounding box.\nVertex: " + vertex + "\nPlane z at vertex: " + planeZGivenXY + "\nBounding Box: "
                  + boundingBox3dInWorld, boundingBox3dInWorld.isInsideInclusive(vertex.getX(), vertex.getY(), planeZGivenXY));
         }
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegion.class, PlanarRegionTest.class);
   }
}
