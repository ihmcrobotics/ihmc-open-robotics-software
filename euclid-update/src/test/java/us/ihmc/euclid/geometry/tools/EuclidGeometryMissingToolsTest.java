package us.ihmc.euclid.geometry.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Vector4D;

import static org.junit.jupiter.api.Assertions.*;

public class EuclidGeometryMissingToolsTest
{
   private static final double EPSILON = 1.0e-10;


   @Test
   public void testGetZOnPlane()
   {
      Point3D point = new Point3D(1.0, 2.0, -3.0);
      Vector3D normal = new Vector3D(0.2, 1.7, 0.4);
      Plane3D plane = new Plane3D(point, normal);

      double x = 2.33;
      double y = 1.97;

      double z = EuclidGeometryMissingTools.getZOnPlane(point, normal, x, y);

      Point3D testPoint = new Point3D(x, y, z);
      assertTrue(plane.distance(testPoint) < 1e-10);

      normal = new Vector3D(0.2, 1.7, 0.0);

      z = EuclidGeometryMissingTools.getZOnPlane(point, normal, x, y);
      assertTrue(Double.isNaN(z));
   }


   @Test
   public void testIsPolygonInside1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(2.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.update();

      ConvexPolygon2D polygonToTest1 = new ConvexPolygon2D();
      polygonToTest1.addVertex(new Point2D(0.1, 0.1));
      polygonToTest1.addVertex(new Point2D(0.2, 0.2));
      polygonToTest1.update();
      assertTrue(EuclidGeometryMissingTools.isPolygonInside(polygonToTest1, polygon));

      ConvexPolygon2D polygonToTest2 = new ConvexPolygon2D(polygon);
      assertTrue(EuclidGeometryMissingTools.isPolygonInside(polygonToTest2, polygon));
      assertTrue(EuclidGeometryMissingTools.isPolygonInside(polygonToTest2, EPSILON, polygon));
      assertFalse(EuclidGeometryMissingTools.isPolygonInside(polygonToTest2, -EPSILON, polygon));

      ConvexPolygon2D polygonToTest3 = new ConvexPolygon2D();
      polygonToTest3.addVertex(new Point2D(0.3, 0.9));
      polygonToTest3.addVertex(new Point2D(0.1, 0.1));
      polygonToTest3.addVertex(new Point2D(1.0, 1.2));
      polygonToTest3.update();
      assertFalse(EuclidGeometryMissingTools.isPolygonInside(polygonToTest3, polygon));

      ConvexPolygon2D polygonToTest4 = new ConvexPolygon2D();
      assertTrue(EuclidGeometryMissingTools.isPolygonInside(polygonToTest4, polygon));

      ConvexPolygon2D polygonToTest5 = new ConvexPolygon2D();
      polygonToTest5.addVertex(new Point2D(-0.1, 0.1));
      polygonToTest5.update();
      assertFalse(EuclidGeometryMissingTools.isPolygonInside(polygonToTest5, polygon));
   }


   @Test
   public void testComputeBoundingBoxIntersection3D()
   {
      BoundingBox3D boundingBox1 = new BoundingBox3D(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      BoundingBox3D boundingBox2 = new BoundingBox3D(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

      assertEquals(1.0, EuclidGeometryMissingTools.computeBoundingBoxVolume3D(boundingBox1), EPSILON, "[FAILED] Volume should be 1.0");
      assertEquals(1.0, EuclidGeometryMissingTools.computeBoundingBoxVolume3D(boundingBox2), EPSILON, "[FAILED] Volume should be 1.0");

      assertEquals(1/3.0, EuclidGeometryMissingTools.computeIntersectionOverUnionOfTwoBoundingBoxes(boundingBox1, boundingBox2), EPSILON, "[FAILED] Intersection should be 1/3.0");
      assertEquals(0.5, EuclidGeometryMissingTools.computeIntersectionOverSmallerOfTwoBoundingBoxes(boundingBox2, boundingBox1), EPSILON, "[FAILED] Intersection should be 0.5");

      boundingBox2 = new BoundingBox3D(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);

      assertEquals(1.0, EuclidGeometryMissingTools.computeBoundingBoxVolume3D(boundingBox2), EPSILON, "[FAILED] Volume should be 1.0");

      assertEquals(1/15.0, EuclidGeometryMissingTools.computeIntersectionOverUnionOfTwoBoundingBoxes(boundingBox1, boundingBox2), EPSILON, "[FAILED] Intersection should be 1/3.0");
      assertEquals( 1/8.0, EuclidGeometryMissingTools.computeIntersectionOverSmallerOfTwoBoundingBoxes(boundingBox2, boundingBox1), EPSILON, "[FAILED] Intersection should be 1/15.0");
   }

   @Test
   public void testPointProjectionsOntoPlane()
   {
      Point3D pointOnPlane = new Point3D(1.0, 1.0, 1.0);
      UnitVector3D planeNormal = new UnitVector3D(1.0, 1.0, 1.0);

      Vector4D plane = new Vector4D(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -pointOnPlane.dot(planeNormal));

      Point3D pointToProject = new Point3D(0.0, 0.0, 0.0);

      Point3D projectedPoint = EuclidGeometryMissingTools.projectPointOntoPlane(plane, pointToProject);


      Point3D expectedProjection = new Point3D(1.0, 1.0, 1.0);

      System.out.println("Projected point: " + projectedPoint);
      System.out.println("Expected projection:  " + expectedProjection);

      assertEquals(0.0, projectedPoint.distance(expectedProjection), EPSILON);
   }


}
