package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class PlanarRegionsListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrivialCase() throws Exception
   {
      // polygons forming a "|"-shaped region.
      List<ConvexPolygon2D> region1ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(5.0, 1.0);
      polygon1.addVertex(5.0, -1.0);
      polygon1.addVertex(-5.0, -1.0);
      polygon1.addVertex(-5.0, 1.0);

      region1ConvexPolygons.add(polygon1);
      for (ConvexPolygon2D convexPolygon : region1ConvexPolygons)
         convexPolygon.update();


      // polygons forming a "--"-shaped region.
      List<ConvexPolygon2D> region2ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(1.0, 5.0);
      polygon2.addVertex(1.0, -5.0);
      polygon2.addVertex(-1.0, -5.0);
      polygon2.addVertex(-1.0, 5.0);

      region2ConvexPolygons.add(polygon2);
      for (ConvexPolygon2D convexPolygon : region2ConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform region1Transform = new RigidBodyTransform();
      RigidBodyTransform region2Transform = new RigidBodyTransform();

      region2Transform.setTranslation(0.0, 0.0, 1.0);

      PlanarRegion planarRegion1 = new PlanarRegion(region1Transform, region1ConvexPolygons);
      PlanarRegion planarRegion2 = new PlanarRegion(region2Transform, region2ConvexPolygons);
      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegion1);
      planarRegions.add(planarRegion2);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      assertEquals("Unexpected number of planar regions.", 2, planarRegionsList.getNumberOfPlanarRegions());
      for (int i = 0; i < 2; i++)
         assertTrue(planarRegionsList.getPlanarRegion(i).epsilonEquals(planarRegions.get(i), 1.0e-10));

      Point2D point2d = new Point2D();
      List<PlanarRegion> result;

      // Do a bunch of trivial queries with findPlanarRegionsContainingPointByProjectionOntoXYPlane(double x, double y)
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(0.0, 0.0);
      assertEquals(2, result.size());
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(2.0, 0.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(-2.0, 0.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(0.0, 2.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(0.0, -2.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(2.0, 2.0);
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(2.0, -2.0);
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(-2.0, -2.0);
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(-2.0, 2.0);
      assertNull(result);

      // Do a bunch of trivial queries with findPlanarRegionsContainingPointByProjectionOntoXYPlane(Point2d point)
      point2d.set(0.0, 0.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertEquals(2, result.size());
      point2d.set(2.0, 0.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point2d.set(-2.0, 0.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point2d.set(0.0, 2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point2d.set(0.0, -2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point2d.set(2.0, 2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertNull(result);
      point2d.set(2.0, -2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertNull(result);
      point2d.set(-2.0, -2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertNull(result);
      point2d.set(-2.0, 2.0);
      result = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point2d);
      assertNull(result);

      Point3D point3d = new Point3D();
      double epsilon = 1.0e-3;

      // Do a bunch of trivial queries with findPlanarRegionsContainingPoint(Point3D point, double epsilon)
      point3d.set(0.0, 0.0, 0.0);
      result = planarRegionsList.findPlanarRegionsContainingPoint(point3d, epsilon);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point3d.set(0.0, 0.0, 1.0);
      result = planarRegionsList.findPlanarRegionsContainingPoint(point3d, epsilon);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point3d.set(0.0, 0.0, 0.5);
      result = planarRegionsList.findPlanarRegionsContainingPoint(point3d, epsilon);
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsContainingPoint(point3d, 0.51);
      assertEquals(2, result.size());

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      // Do a bunch of trivial queries with findPlanarRegionsIntersectingPolygon(ConvexPolygon2d convexPolygon)
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(convexPolygon);
      assertEquals(2, result.size());
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(2.0, 0.0, convexPolygon));
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(-2.0, 0.0, convexPolygon));
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(0.0, 2.0, convexPolygon));
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(0.0, -2.0, convexPolygon));
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(2.0, 2.0, convexPolygon));
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(2.0, -2.0, convexPolygon));
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(-2.0, -2.0, convexPolygon));
      assertNull(result);
      result = planarRegionsList.findPlanarRegionsIntersectingPolygon(PlanarRegionTest.translateConvexPolygon(-2.0, 2.0, convexPolygon));
      assertNull(result);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionsList.class, PlanarRegionsListTest.class);
   }
}
