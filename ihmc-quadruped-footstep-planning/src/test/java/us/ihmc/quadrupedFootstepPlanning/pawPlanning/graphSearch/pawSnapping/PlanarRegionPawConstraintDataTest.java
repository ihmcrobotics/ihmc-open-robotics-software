package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import static us.ihmc.robotics.Assert.assertTrue;

public class PlanarRegionPawConstraintDataTest
{
   @Test
   public void testSimpleRectangle()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(0.2, 1.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegionPawConstraintData planarRegionPawConstraintData = new PlanarRegionPawConstraintData(scaler, planarRegionsList.getPlanarRegion(0), true, 0.2);
      ConvexPolygon2DReadOnly scaledRegionPolygon = planarRegionPawConstraintData.getScaledRegionPolygon(new Point2D(0.0, 0.0));

      Point2D projectionPoint = new Point2D();
      double epsilon = 1e-3;

      projectionPoint.set(0.0, 1.0);
      scaledRegionPolygon.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(new Point2D(0.0, 0.4), epsilon));

      projectionPoint.set(0.0, -1.0);
      scaledRegionPolygon.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(new Point2D(0.0, -0.4), epsilon));

      projectionPoint.set(1.0, 0.0);
      scaledRegionPolygon.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(new Point2D(0.0, 0.0), epsilon));

      projectionPoint.set(-1.0, 0.0);
      scaledRegionPolygon.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(new Point2D(0.0, 0.0), epsilon));
   }

   @Test
   public void testTiming()
   {
      
   }
}
