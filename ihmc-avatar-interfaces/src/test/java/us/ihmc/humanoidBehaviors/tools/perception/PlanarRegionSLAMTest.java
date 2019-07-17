package us.ihmc.humanoidBehaviors.tools.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

class PlanarRegionSLAMTest
{

   @Test
   void testFindHighConfidencePairs()
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      PlanarRegion unitSquare = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      PlanarRegion anotherUnitSquare = createASingleSquareCenteredAtOrigin(1.0, 1.0);

      map.addPlanarRegion(unitSquare);
      newData.addPlanarRegion(anotherUnitSquare);

      PairList<PlanarRegion, PlanarRegion> highConfidencePairs = PlanarRegionSLAM.findHighConfidencePairs(map, newData);

      assertEquals(1, highConfidencePairs.size());
      ImmutablePair<PlanarRegion, PlanarRegion> pair = highConfidencePairs.get(0);

      assertTrue(unitSquare == pair.getLeft());
      assertTrue(anotherUnitSquare == pair.getRight());
   }

   private PlanarRegion createASingleSquareCenteredAtOrigin(double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(new Vector3D(), 0.0, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquare(Vector3D translation, double yaw, double pitch, double roll, Point2D minimumPoint, Point2D maximumPoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslation(translation);
      transformToWorld.setRotationYawPitchRoll(yaw, pitch, roll);

      ArrayList<Point2D> vertices = new ArrayList<Point2D>();
      vertices.add(new Point2D(minimumPoint));
      vertices.add(new Point2D(minimumPoint.getX(), maximumPoint.getY()));
      vertices.add(new Point2D(maximumPoint));
      vertices.add(new Point2D(maximumPoint.getX(), minimumPoint.getY()));

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      PlanarRegion square = new PlanarRegion(transformToWorld, convexPolygon);
      return square;

   }

}
