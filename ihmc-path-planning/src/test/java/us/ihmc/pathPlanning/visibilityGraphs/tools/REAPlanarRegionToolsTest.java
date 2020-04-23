package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.planarRegion.REAPlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class REAPlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   public static Point3D toWorld(Point2D point2D, Transform transformToWorld)
   {
      Point3D inWorld = new Point3D(point2D);
      transformToWorld.transform(inWorld);
      return inWorld;
   }

   @Test
   public void testTruncatePlanarRegionIfIntersectingWithPlane() throws Exception
   {
      Point3D groundOrigin = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(0.0, 0.0, -0.001);
      Vector3D squareNormal = new Vector3D(0.0, -1.0, 0.0);
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      List<Point2D> concaveHullVertices = new ArrayList<>();
      concaveHullVertices.add(new Point2D(0.0, 0.0));
      concaveHullVertices.add(new Point2D(0.0, squareSide));
      concaveHullVertices.add(new Point2D(squareSide, squareSide));
      concaveHullVertices.add(new Point2D(squareSide, 0.0));

      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices)));
      PlanarRegion verticalSquare = new PlanarRegion(squarePose, concaveHullVertices, convexPolygons);

      Point3D[] expectedVerticesInWorld = concaveHullVertices.stream().map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);
      expectedVerticesInWorld[0].addZ(0.001);
      expectedVerticesInWorld[3].addZ(0.001);

      PlanarRegion truncatedSquare = REAPlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, verticalSquare, 0.05, null);
      RigidBodyTransform truncatedTransform = new RigidBodyTransform();
      truncatedSquare.getTransformToWorld(truncatedTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(squarePose, truncatedTransform, EPSILON);

      Point3D[] actualVerticesInWorld = truncatedSquare.getConcaveHull().stream().map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);

      assertEquals(expectedVerticesInWorld.length, actualVerticesInWorld.length);

      for (int i = 0; i < expectedVerticesInWorld.length; i++)
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedVerticesInWorld[i], actualVerticesInWorld[i], EPSILON);
   }

   @Test
   public void testTruncatePlanarRegionIfIntersectingWithPlaneTwo() throws Exception
   {
      Point3D groundOrigin = new Point3D(4.25, 8.5, 0.0);
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(8.5, 8.5, 0.0);
      Vector3D squareNormal = new Vector3D(-0.1, 0.1, 0.9899);
      squareNormal.normalize();
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      Point2D[] concaveHullVertices = {new Point2D(-squareSide/2.0, squareSide/2.0), new Point2D(squareSide/2.0, squareSide/2.0), new Point2D(squareSide/2.0, -squareSide/2.0), new Point2D(-squareSide/2.0, -squareSide/2.0)};
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices)));
      PlanarRegion rotatedSquare = new PlanarRegion(squarePose, Arrays.asList(concaveHullVertices), convexPolygons);

      PlanarRegion truncatedSquare = REAPlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, rotatedSquare, 0.05, null);

      //      TODO: Finish this test up with some asserts and more cases.
      //      System.out.println(truncatedSquare);

   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(REAPlanarRegionTools.class, REAPlanarRegionToolsTest.class);
   }
}