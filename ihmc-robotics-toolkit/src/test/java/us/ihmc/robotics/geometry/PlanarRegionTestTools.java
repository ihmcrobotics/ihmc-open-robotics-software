package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class PlanarRegionTestTools
{
   public static void assertPlanarRegionsEqual(PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);
      EuclidCoreTestTools.assertEquals(expectedTransform, actualTransform, epsilon);

      assertEquals(expected.getConcaveHullSize(), actual.getConcaveHullSize());

      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertEquals(expected.getConcaveHullVertex(i), actual.getConcaveHullVertex(i), epsilon);
      }

      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());

      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D expectedConvexPolygon = expected.getConvexPolygon(i);
         ConvexPolygon2D actualConvexPolygon = actual.getConvexPolygon(i);
         assertEquals(expectedConvexPolygon.getNumberOfVertices(), actualConvexPolygon.getNumberOfVertices());
         assertTrue(expectedConvexPolygon.epsilonEquals(actualConvexPolygon, epsilon));
      }
   }

   public static void assertPlanarRegionsGeometricallyEqual(PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);
      EuclidCoreTestTools.assertGeometricallyEquals(expectedTransform, actualTransform, epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(expected.getNormal(), actual.getNormal(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(expected.getPoint(), actual.getPoint(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(expected.getBoundingBox3dInWorld(), actual.getBoundingBox3dInWorld(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(expected.getConvexHull(), actual.getConvexHull(), epsilon);

      assertEquals(expected.getConcaveHullSize(), actual.getConcaveHullSize());

      // TODO make this handle the sort problem
      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertGeometricallyEquals(expected.getConcaveHullVertex(i), actual.getConcaveHullVertex(i), epsilon);
      }

      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());

      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D expectedConvexPolygon = expected.getConvexPolygon(i);
         ConvexPolygon2D actualConvexPolygon = actual.getConvexPolygon(i);

         EuclidCoreTestTools.assertGeometricallyEquals(expectedConvexPolygon, actualConvexPolygon, epsilon);
      }
   }
}

