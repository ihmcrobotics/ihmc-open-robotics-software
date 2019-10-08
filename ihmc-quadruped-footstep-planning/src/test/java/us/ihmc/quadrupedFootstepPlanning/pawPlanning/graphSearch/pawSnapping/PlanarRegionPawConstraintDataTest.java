package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import java.util.Random;

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

      PlanarRegionPawConstraintData planarRegionPawConstraintData = new PlanarRegionPawConstraintData(scaler, planarRegionsList.getPlanarRegion(0), true, 0.2, -1.0);
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

   private static void testTiming()
   {
      Random random = new Random(3290);

      int numberOfRegions = 1000;
      double[] successfulProjectionDistance = new double[numberOfRegions];
      double[] unsuccessfulProjectionDistances = new double[numberOfRegions];

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      for (int i = 0; i < numberOfRegions; i++)
      {
         generator.translateThenRotate(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         double sideX = EuclidCoreRandomTools.nextDouble(random, 0.1, 2.0);
         double sideY = EuclidCoreRandomTools.nextDouble(random, 0.1, 2.0);
         generator.addRectangle(sideX, sideY);

         successfulProjectionDistance[i] = EuclidCoreRandomTools.nextDouble(random, 1e-2, 0.5 * Math.min(sideX, sideY) - 1e-2);
         unsuccessfulProjectionDistances[i] = EuclidCoreRandomTools.nextDouble(random, 0.5 * Math.min(sideX, sideY) + 1e-2, Math.max(sideX, sideY) - 1e-2);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      long start, stop;

      start = System.nanoTime();
      for (int i = 0; i < numberOfRegions; i++)
      {
         new PlanarRegionPawConstraintData(scaler, planarRegionsList.getPlanarRegion(i), true, successfulProjectionDistance[i], -1.0);
      }
      stop = System.nanoTime();
      System.out.println("Average time taken not using fallback projection: " + (Conversions.nanosecondsToSeconds(stop - start) / numberOfRegions) + " sec");

      start = System.nanoTime();
      for (int i = 0; i < numberOfRegions; i++)
      {
         new PlanarRegionPawConstraintData(scaler, planarRegionsList.getPlanarRegion(i), true, unsuccessfulProjectionDistances[i], -1.0);
      }
      stop = System.nanoTime();
      System.out.println("Average time taken using fallback projection: " + (Conversions.nanosecondsToSeconds(stop - start) / numberOfRegions) + " sec");

   }

   public static void main(String[] args)
   {
      testTiming();
   }
}
