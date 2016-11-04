package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class PlanarRegionTest
{
   @Test
   public void testWithLShapedPlanarRegionWithIdentityTransform()
   {
      // polygons forming a L-shaped region.
      List<ConvexPolygon2d> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d polygon1 = new ConvexPolygon2d();
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(-1.0, 1.0);
      ConvexPolygon2d polygon2 = new ConvexPolygon2d();
      polygon2.addVertex(3.0, 1.0);
      polygon2.addVertex(3.0, -1.0);
      polygon2.addVertex(1.0, -1.0);
      polygon2.addVertex(1.0, 1.0);
      ConvexPolygon2d polygon3 = new ConvexPolygon2d();
      polygon3.addVertex(1.0, 3.0);
      polygon3.addVertex(1.0, 1.0);
      polygon3.addVertex(-1.0, 1.0);
      polygon3.addVertex(-1.0, 3.0);

      regionConvexPolygons.add(polygon1);
      regionConvexPolygons.add(polygon2);
      regionConvexPolygons.add(polygon3);
      for (ConvexPolygon2d convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      assertEquals("Wrong number of convex polygons in the region.", 3, planarRegion.getNumberOfConvexPolygons());
      for (int i = 0; i < 3; i++)
         assertTrue("Unexpected region polygon.", regionConvexPolygons.get(i).epsilonEquals(planarRegion.getConvexPolygon(i), 1.0e-10));

      Vector3d actualNormal = new Vector3d();
      planarRegion.getNormal(actualNormal);
      JUnitTools.assertVector3dEquals("Wrong region normal.", new Vector3d(0.0, 0.0, 1.0), actualNormal, 1.0e-10);
      Point3d actualOrigin = new Point3d();
      planarRegion.getPointInRegion(actualOrigin);
      JUnitTools.assertPoint3dEquals("Wrong region origin.", new Point3d(), actualOrigin, 1.0e-10);
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(actualTransform);
      assertTrue("Wrong region transform to world.", regionTransform.epsilonEquals(actualTransform, 1.0e-10));

      Point2d point2d = new Point2d();

      // Do a bunch of trivial queries with isPointInside(Point2d) method.
      point2d.set(0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(2.0, 0.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(0.0, 2.0);
      assertTrue(planarRegion.isPointInside(point2d));
      point2d.set(2.0, 2.0);
      assertFalse(planarRegion.isPointInside(point2d));

      Point3d point3d = new Point3d();
      double epsilon = 1.0e-3;
      // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point in plane
      point3d.set(0.0, 0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 0.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 2.0, 0.0);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 2.0, 0.0);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point below plane
      point3d.set(0.0, 0.0, -0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 0.0, -0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 2.0, -0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 0.0, -1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 0.0, -1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 2.0, -1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point above plane
      point3d.set(0.0, 0.0, 0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 0.0, 0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 2.0, 0.5 * epsilon);
      assertTrue(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 0.0, 1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(2.0, 0.0, 1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));
      point3d.set(0.0, 2.0, 1.5 * epsilon);
      assertFalse(planarRegion.isPointInside(point3d, epsilon));

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

      // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point3d) method.
      point3d.set(0.0, 0.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(2.0, 0.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(0.0, 2.0, Double.POSITIVE_INFINITY);
      assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));
      point3d.set(2.0, 2.0, Double.POSITIVE_INFINITY);
      assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d));

      ConvexPolygon2d convexPolygon = new ConvexPolygon2d();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      // Do a bunch of trivial queris with isPolygonIntersecting(ConvexPolygon2d) method.
      assertTrue(planarRegion.isPolygonIntersecting(convexPolygon));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(2.0, 0.0, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(0.0, 2.0, convexPolygon)));
      assertFalse(planarRegion.isPolygonIntersecting(translateConvexPolygon(2.0, 2.0, convexPolygon)));
      assertFalse(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.21, 1.21, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.09, 1.09, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.21, 1.09, convexPolygon)));
      assertTrue(planarRegion.isPolygonIntersecting(translateConvexPolygon(1.09, 1.21, convexPolygon)));
   }

   @Test
   public void testWithLShapedPlanarRegionWithRandomTransform()
   {
      Random random = new Random(42L);

      // polygons forming a L-shaped region.
      List<ConvexPolygon2d> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d polygon1 = new ConvexPolygon2d();
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(-1.0, 1.0);
      ConvexPolygon2d polygon2 = new ConvexPolygon2d();
      polygon2.addVertex(3.0, 1.0);
      polygon2.addVertex(3.0, -1.0);
      polygon2.addVertex(1.0, -1.0);
      polygon2.addVertex(1.0, 1.0);
      ConvexPolygon2d polygon3 = new ConvexPolygon2d();
      polygon3.addVertex(1.0, 3.0);
      polygon3.addVertex(1.0, 1.0);
      polygon3.addVertex(-1.0, 1.0);
      polygon3.addVertex(-1.0, 3.0);

      regionConvexPolygons.add(polygon1);
      regionConvexPolygons.add(polygon2);
      regionConvexPolygons.add(polygon3);
      for (ConvexPolygon2d convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int iteration = 0; iteration < 10; iteration++)
      {
         Quat4d orientation = RandomTools.generateRandomQuaternion(random, Math.toRadians(45.0));
         Vector3d translation = RandomTools.generateRandomVector(random, 10.0);
         RigidBodyTransform regionTransform = new RigidBodyTransform(orientation, translation);
         ReferenceFrame localFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("local", worldFrame, regionTransform);
         PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

         assertEquals("Wrong number of convex polygons in the region.", 3, planarRegion.getNumberOfConvexPolygons());
         for (int i = 0; i < 3; i++)
            assertTrue("Unexpected region polygon.", regionConvexPolygons.get(i).epsilonEquals(planarRegion.getConvexPolygon(i), 1.0e-10));

         Vector3d expectedNormal = new Vector3d(0.0, 0.0, 1.0);
         regionTransform.transform(expectedNormal);
         Vector3d actualNormal = new Vector3d();
         planarRegion.getNormal(actualNormal);
         JUnitTools.assertVector3dEquals("Wrong region normal.", expectedNormal, actualNormal, 1.0e-10);
         Point3d expectedOrigin = new Point3d();
         regionTransform.transform(expectedOrigin);
         Point3d actualOrigin = new Point3d();
         planarRegion.getPointInRegion(actualOrigin);
         JUnitTools.assertPoint3dEquals("Wrong region origin.", expectedOrigin, actualOrigin, 1.0e-10);
         RigidBodyTransform actualTransform = new RigidBodyTransform();
         planarRegion.getTransformToWorld(actualTransform);
         assertTrue("Wrong region transform to world.", regionTransform.epsilonEquals(actualTransform, 1.0e-10));

         FramePoint2d point2d = new FramePoint2d();

         // Do a bunch of trivial queries with isPointInside(Point2d) method.
         point2d.setIncludingFrame(localFrame, 0.0, 0.0);
         assertTrue(planarRegion.isPointInside(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 2.0, 0.0);
         assertTrue(planarRegion.isPointInside(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 0.0, 2.0);
         assertTrue(planarRegion.isPointInside(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 2.0, 2.0);
         assertFalse(planarRegion.isPointInside(point2d.getPoint()));

         FramePoint point3d = new FramePoint();
         double epsilon = 1.0e-3;
         // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point in plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point below plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, -0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, -0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, -0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, -1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, -1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, -1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         // Do a bunch of trivial queries with isPointInside(Point3d, double) method. Point above plane
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertTrue(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 1.5 * epsilon);
         point3d.changeFrame(worldFrame);
         assertFalse(planarRegion.isPointInside(point3d.getPoint(), epsilon));

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
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 2.0, 0.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 0.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getPoint()));
         point2d.setIncludingFrame(localFrame, 2.0, 2.0);
         point2d.changeFrameAndProjectToXYPlane(worldFrame);
         assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point2d.getPoint()));

         // Do a bunch of trivial queries with isPointInsideByProjectionOntoXYPlane(Point3d) method.
         point3d.setIncludingFrame(localFrame, 0.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d.getPoint()));
         point3d.setIncludingFrame(localFrame, 2.0, 0.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d.getPoint()));
         point3d.setIncludingFrame(localFrame, 0.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertTrue(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d.getPoint()));
         point3d.setIncludingFrame(localFrame, 2.0, 2.0, 0.0);
         point3d.changeFrame(worldFrame);
         point3d.setZ(Double.POSITIVE_INFINITY);
         assertFalse(planarRegion.isPointInsideByProjectionOntoXYPlane(point3d.getPoint()));

         ConvexPolygon2d convexPolygon = new ConvexPolygon2d();
         convexPolygon.addVertex(0.2, 0.2);
         convexPolygon.addVertex(0.2, -0.2);
         convexPolygon.addVertex(-0.2, -0.2);
         convexPolygon.addVertex(-0.2, 0.2);
         convexPolygon.update();

         // Do a bunch of trivial queris with isPolygonIntersecting(ConvexPolygon2d) method.
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, convexPolygon)));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(2.0, 0.0, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(0.0, 2.0, convexPolygon))));
         assertFalse(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(2.0, 2.0, convexPolygon))));
         assertFalse(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.21, 1.21, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.09, 1.09, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.21, 1.09, convexPolygon))));
         assertTrue(planarRegion.isPolygonIntersecting(transformConvexPolygon(regionTransform, translateConvexPolygon(1.09, 1.21, convexPolygon))));
      }
   }

   static ConvexPolygon2d translateConvexPolygon(double tx, double ty, ConvexPolygon2d convexPolygon)
   {
      ConvexPolygon2d translatedConvexPolygon = new ConvexPolygon2d();
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         double x = convexPolygon.getVertex(i).getX() + tx;
         double y = convexPolygon.getVertex(i).getY() + ty;
         translatedConvexPolygon.addVertex(x, y);
      }
      translatedConvexPolygon.update();
      return translatedConvexPolygon;
   }

   private ConvexPolygon2d transformConvexPolygon(RigidBodyTransform transform, ConvexPolygon2d convexPolygon)
   {
      ConvexPolygon2d transformedConvexPolygon = new ConvexPolygon2d();
      Point3d transformedVertex = new Point3d();

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = convexPolygon.getVertex(i);
         transformedVertex.set(vertex.getX(), vertex.getY(), 0.0);
         transform.transform(transformedVertex);
         transformedConvexPolygon.addVertex(transformedVertex.getX(), transformedVertex.getY());
      }
      transformedConvexPolygon.update();
      return transformedConvexPolygon;
   }
}
