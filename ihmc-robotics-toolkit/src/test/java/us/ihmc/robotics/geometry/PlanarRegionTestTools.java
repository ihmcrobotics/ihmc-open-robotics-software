package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import static org.junit.Assert.assertEquals;

public class PlanarRegionTestTools
{
   public static void assertPlanarReginsEqual(PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, epsilon);

      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         EuclidGeometryTestTools.assertConvexPolygon2DEquals(expected.getConvexPolygon(i), actual.getConvexPolygon(i), epsilon);
      }

      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expected.getConvexHull(), actual.getConvexHull(), epsilon);
   }

   public static void assertPlanarReginsGeometricallyEqual(PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);

      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(expectedTransform, actualTransform, epsilon);

      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(expected.getConvexPolygon(i), actual.getConvexPolygon(i), epsilon);
      }

      EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(expected.getConvexHull(), actual.getConvexHull(), epsilon);
   }
}
