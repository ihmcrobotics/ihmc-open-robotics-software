package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.HullFace;
import us.ihmc.robotics.geometry.QuickHull3dWrapper;
import us.ihmc.robotics.geometry.transformables.Pose;

public class QuickHull3DWrapperTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimplexHull()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 0, 1));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 4);
      assertTrue(quickHull.getNumFaces() == 4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtraPointInSimplex()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 0, 1));
      points.add(new Point3D(0.35, 0.35, 0));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 4);
      assertTrue(quickHull.getNumFaces() == 4);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtraPointOnSimplex()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 0, 1));
      points.add(new Point3D(0.25, 0.25, 0.5));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 4);
      assertTrue(quickHull.getNumFaces() == 4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFivePointHull()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 0, 1));
      points.add(new Point3D(0.25, 0.25, 1.0));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 5);
      assertTrue(quickHull.getNumFaces() == 6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtraPointApproximatelyOnSimplex()
   {
      double testTolerance = 1e-15;
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 0, 1));
      points.add(new Point3D(0.25, 0.25, 0.5 + testTolerance));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      if (testTolerance < quickHull.getDistanceTolerance())
      {
         assertTrue(quickHull.getNumVertices() == 4);
         assertTrue(quickHull.getNumFaces() == 4);
      }
      else
      {
         assertTrue(quickHull.getNumVertices() == 5);
         assertTrue(quickHull.getNumFaces() == 6);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCubeHull()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(0, 0, 0));
      points.add(new Point3D(0, 0, 1));
      points.add(new Point3D(0, 1, 0));
      points.add(new Point3D(0, 1, 1));
      points.add(new Point3D(1, 0, 0));
      points.add(new Point3D(1, 0, 1));
      points.add(new Point3D(1, 1, 0));
      points.add(new Point3D(1, 1, 1));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 8);
      assertTrue(quickHull.getNumFaces() == 6);

      List<HullFace> faces = quickHull.getFaces();
      for (HullFace face : faces)
      {
         assertEquals(1.0, face.getArea(), 1e-15);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrapezoidalPrismHull()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(2, 2, 0));
      points.add(new Point3D(-2, 2, 0));
      points.add(new Point3D(-2, -2, 0));
      points.add(new Point3D(2, -2, 0));
      points.add(new Point3D(1, 1, 1));
      points.add(new Point3D(-1, 1, 1));
      points.add(new Point3D(-1, -1, 1));
      points.add(new Point3D(1, -1, 1));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 8);
      assertTrue(quickHull.getNumFaces() == 6);

      List<HullFace> faces = quickHull.getFaces();
      Plane3d facePlane = new Plane3d();
      Vector3D planeNormal = new Vector3D();
      for (HullFace face : faces)
      {
         face.getPlane(facePlane);
         facePlane.getNormal(planeNormal);

         if (planeNormal.epsilonEquals(new Vector3D(0, 0, 1), 1e-14))
         {
            assertEquals(0.0, face.getSlopeAngle(), 1e-14);
            assertEquals(4.0, face.getArea(), 1e-14);
         }
         else if (planeNormal.epsilonEquals(new Vector3D(0, 0, -1), 1e-14))
         {
            assertEquals(Math.PI, face.getSlopeAngle(), 1e-14);
            assertEquals(16.0, face.getArea(), 1e-14);
         }
         else
         {
            assertEquals(Math.PI / 4, face.getSlopeAngle(), 1e-14);
            assertEquals(3.0 * Math.sqrt(2), face.getArea(), 1e-14);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFaceToPolygonTrapezoidalPrismHull()
   {
      List<Point3D> points = new ArrayList<Point3D>();
      points.add(new Point3D(2, 2, 0));
      points.add(new Point3D(-2, 2, 0));
      points.add(new Point3D(-2, -2, 0));
      points.add(new Point3D(2, -2, 0));
      points.add(new Point3D(1, 1, 1));
      points.add(new Point3D(-1, 1, 1));
      points.add(new Point3D(-1, -1, 1));
      points.add(new Point3D(1, -1, 1));

      QuickHull3dWrapper quickHull = new QuickHull3dWrapper(points);

      assertTrue(quickHull.getNumVertices() == 8);
      assertTrue(quickHull.getNumFaces() == 6);

      List<HullFace> faces = quickHull.getFaces();

      List<Point3D> vertices = new ArrayList<Point3D>();
      ConvexPolygon2d currentPolygon = new ConvexPolygon2d();
      Pose polygonPose = new Pose(new Point3D(), new Quaternion());
      faces.get(0).get2DPolygonAndPose(currentPolygon, polygonPose);
      vertices.addAll(faces.get(0).getPoints());

      double sideLength = Math.sqrt(2) * 2;
      double tolerance = 1e-10;
      System.out.println(vertices.get(2));
      assertTrue(vertices.get(0).epsilonEquals(new Point3D(2, -2, 0), tolerance));
      assertTrue(vertices.get(1).epsilonEquals(new Point3D(2, 2, 0), tolerance));
      assertTrue(vertices.get(2).epsilonEquals(new Point3D(-2, 2, 0), tolerance));
      assertTrue(vertices.get(3).epsilonEquals(new Point3D(-2, -2, 0), tolerance));

      assertTrue(currentPolygon.getVertex(0).epsilonEquals(new Point2D(sideLength, 0.0), tolerance));
      assertTrue(currentPolygon.getVertex(1).epsilonEquals(new Point2D(0.0, -sideLength), tolerance));
      assertTrue(currentPolygon.getVertex(2).epsilonEquals(new Point2D(-sideLength, 0.0), tolerance));
      assertTrue(currentPolygon.getVertex(3).epsilonEquals(new Point2D(0.0, sideLength), tolerance));

      assertTrue(polygonPose.getPoint().epsilonEquals(new Point3D(), tolerance));
      Assert.assertEquals(polygonPose.getOrientation().getYaw(), -Math.PI / 4, tolerance);
   }
}
