package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import static us.ihmc.robotics.Assert.*;

public class PlanarRegionTestTools
{
   public static void assertPlanarRegionsEqual(PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(expectedTransform, actualTransform, epsilon);

      assertEquals(expected.getConcaveHullSize(), actual.getConcaveHullSize());

      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertTuple2DEquals(expected.getConcaveHullVertex(i), actual.getConcaveHullVertex(i), epsilon);
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
}

