package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class VisibilityGraphsParametersReadOnlyTest
{
   private static final double iters = 1000;

   @Test
   public void testFilterConnections()
   {
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      double preferredConnectionDistance = parameters.getPreferredNavigableExtrusionDistance();
      double interRegionDistance = parameters.getMaxInterRegionConnectionLength();
      double connectionDistance = parameters.getNavigableExtrusionDistance();

      InterRegionConnectionFilter nonPreferredToNonPreferred = parameters.getInterRegionConnectionFilter();
      InterRegionConnectionFilter preferredToNonPreferred = parameters.getPreferredToNonPreferredInterRegionConnectionFilter();
      InterRegionConnectionFilter preferredToPreferred = parameters.getPreferredToPreferredInterRegionConnectionFilter();

      double maxNonPreferredToNonPreferredDistance = 2.0 * connectionDistance + interRegionDistance;
      double maxPreferredToNonPreferredDistance = preferredConnectionDistance + connectionDistance + interRegionDistance;
      double maxPreferredToPreferredDistance = 2.0 * preferredConnectionDistance + interRegionDistance;
      double heightLimit = parameters.getTooHighToStepDistance();

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         ConnectionPoint3D sourcePoint = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), -1);
         Vector2D randomVector = EuclidCoreRandomTools.nextVector2D(random, -1.0, 1.0);
         randomVector.normalize();

         // test well within distance limits
         double fractionToLimit = 0.9;
         Vector2D nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         Vector2D preferredToNonPreferredOffset = new Vector2D(randomVector);
         Vector2D preferredToPreferredOffset = new Vector2D(randomVector);
         double heightOffset = 0.0;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, true);


         // test barely within distance limits
         fractionToLimit = 0.99;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = 0.0;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, true);


         // test right on distance limits
         fractionToLimit = 1.0 - 1e-4;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = 0.0;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, true);

         // test just over distance limits
         fractionToLimit = 1.01;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = 0.0;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, false);

         // test well over distance limits
         fractionToLimit = 1.1;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = 0.0;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, false);


         // test almost too high
         fractionToLimit = 0.99;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = fractionToLimit * heightLimit;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, true);

         // test almost too low
         fractionToLimit = 0.99;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = -fractionToLimit * heightLimit;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, true);

         // test too high
         fractionToLimit = 1.01;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = fractionToLimit * heightLimit;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, false);

         // test too low
         fractionToLimit = 1.01;
         nonPreferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToNonPreferredOffset = new Vector2D(randomVector);
         preferredToPreferredOffset = new Vector2D(randomVector);
         heightOffset = -fractionToLimit * heightLimit;

         nonPreferredToNonPreferredOffset.scale(fractionToLimit * maxNonPreferredToNonPreferredDistance);
         preferredToNonPreferredOffset.scale(fractionToLimit * maxPreferredToNonPreferredDistance);
         preferredToPreferredOffset.scale(fractionToLimit * maxPreferredToPreferredDistance);

         runTest(nonPreferredToNonPreferred, preferredToNonPreferred, preferredToPreferred, sourcePoint, heightOffset, nonPreferredToNonPreferredOffset,
                 preferredToNonPreferredOffset, preferredToPreferredOffset, false);
      }

   }

   private static void runTest(InterRegionConnectionFilter nonPreferredToNonPreferredFilter,
                               InterRegionConnectionFilter preferredToNonPreferredFilter,
                               InterRegionConnectionFilter preferredToPreferredFilter,
                               ConnectionPoint3D source, double heightOffset, Vector2DReadOnly nonPreferredToNonPreferredOffset,
                               Vector2DReadOnly preferredToNonPreferredOffset, Vector2DReadOnly preferredToPreferredOffset, boolean shouldBeValid)
   {
      Point3D nonPreferredToNonPreferredTargetPoint = new Point3D(source.getX(), source.getY(), source.getZ());
      Point3D preferredToNonPreferredTargetPoint = new Point3D(source.getX(), source.getY(), source.getZ());
      Point3D preferredToPreferredTargetPoint = new Point3D(source.getX(), source.getY(), source.getZ());

      nonPreferredToNonPreferredTargetPoint.addX(nonPreferredToNonPreferredOffset.getX());
      nonPreferredToNonPreferredTargetPoint.addY(nonPreferredToNonPreferredOffset.getY());
      nonPreferredToNonPreferredTargetPoint.addZ(heightOffset);

      preferredToNonPreferredTargetPoint.addX(preferredToNonPreferredOffset.getX());
      preferredToNonPreferredTargetPoint.addY(preferredToNonPreferredOffset.getY());
      preferredToNonPreferredTargetPoint.addZ(heightOffset);

      preferredToPreferredTargetPoint.addX(preferredToPreferredOffset.getX());
      preferredToPreferredTargetPoint.addY(preferredToPreferredOffset.getY());
      preferredToPreferredTargetPoint.addZ(heightOffset);

      assertEquals(shouldBeValid, nonPreferredToNonPreferredFilter.isConnectionValid(source, new ConnectionPoint3D(nonPreferredToNonPreferredTargetPoint, -1)));
      assertEquals(shouldBeValid, preferredToNonPreferredFilter.isConnectionValid(source, new ConnectionPoint3D(preferredToNonPreferredTargetPoint, -1)));
      assertEquals(shouldBeValid, preferredToPreferredFilter.isConnectionValid(source, new ConnectionPoint3D(preferredToPreferredTargetPoint, -1)));
   }

   @Test
   public void testFilterRandomConnections()
   {
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      double preferredConnectionDistance = parameters.getPreferredNavigableExtrusionDistance();
      double interRegionDistance = parameters.getMaxInterRegionConnectionLength();
      double connectionDistance = parameters.getNavigableExtrusionDistance();

      InterRegionConnectionFilter nonPreferredToNonPreferredFilter = parameters.getInterRegionConnectionFilter();
      InterRegionConnectionFilter preferredToNonPreferred = parameters.getPreferredToNonPreferredInterRegionConnectionFilter();
      InterRegionConnectionFilter preferredToPreferred = parameters.getPreferredToPreferredInterRegionConnectionFilter();

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         Point3D sourcePoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector2D randomVector = EuclidCoreRandomTools.nextVector2D(random, -1.0, 1.0);
         randomVector.normalize();
         double randomScalar = RandomNumbers.nextDouble(random, 0.0, 1.0);
         double heightOffset = RandomNumbers.nextDouble(random, parameters.getTooHighToStepDistance());

         Vector2D offsetVector = new Vector2D(randomVector);
         offsetVector.scale(randomScalar * (2.0 * connectionDistance + interRegionDistance));

         Point3D targetPoint = new Point3D(sourcePoint);
         targetPoint.addX(offsetVector.getX());
         targetPoint.addY(offsetVector.getY());
         targetPoint.addZ(heightOffset);

         ConnectionPoint3D source = new ConnectionPoint3D(sourcePoint, -1);
         ConnectionPoint3D target = new ConnectionPoint3D(targetPoint, -1);

         String message = "Source " + sourcePoint + " is too far from target " + targetPoint + ".\n It is at a distance of " + sourcePoint.distanceXY(targetPoint) +
               ", which must be less than " + (interRegionDistance + 2.0 * connectionDistance) ;
         assertTrue(message, nonPreferredToNonPreferredFilter.isConnectionValid(source, target));

         offsetVector.set(randomVector);
         offsetVector.scale(randomScalar * (connectionDistance + preferredConnectionDistance + interRegionDistance));

         targetPoint = new Point3D(sourcePoint);
         targetPoint.addX(offsetVector.getX());
         targetPoint.addY(offsetVector.getY());
         targetPoint.addZ(heightOffset);

         target = new ConnectionPoint3D(targetPoint, -1);

         message = "Source " + sourcePoint + " is too far from target " + targetPoint + ".\n It is at a distance of " + sourcePoint.distanceXY(targetPoint) +
               ", which must be less than " + (interRegionDistance + connectionDistance + preferredConnectionDistance);
         assertTrue(message, preferredToNonPreferred.isConnectionValid(source, target));

         offsetVector.set(randomVector);
         offsetVector.scale(randomScalar * (connectionDistance + 2.0 * preferredConnectionDistance));

         targetPoint = new Point3D(sourcePoint);
         targetPoint.addX(offsetVector.getX());
         targetPoint.addY(offsetVector.getY());
         targetPoint.addZ(heightOffset);

         target = new ConnectionPoint3D(targetPoint, -1);

         message = "Source " + sourcePoint + " is too far from target " + targetPoint + ".\n It is at a distance of " + sourcePoint.distanceXY(targetPoint) +
               ", which must be less than " + (interRegionDistance + 2.0 * preferredConnectionDistance);
         assertTrue(message, preferredToPreferred.isConnectionValid(source, target));
      }
   }
}
