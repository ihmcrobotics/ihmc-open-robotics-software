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
      double interRegionDistance = parameters.getMaxInterRegionConnectionLength();
      double connectionDistance = parameters.getNavigableExtrusionDistance();

      InterRegionConnectionFilter interRegionConnectionFilter = parameters.getInterRegionConnectionFilter();

      double maxDistance = 2.0 * connectionDistance + interRegionDistance;
      double heightLimit = parameters.getTooHighToStepDistance();

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         ConnectionPoint3D sourcePoint = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), -1);
         Vector2D randomVector = EuclidCoreRandomTools.nextVector2D(random, -1.0, 1.0);
         randomVector.normalize();

         // test well within distance limits
         double fractionToLimit = 0.9;
         Vector2D offset = new Vector2D(randomVector);
         double heightOffset = 0.0;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, true);


         // test barely within distance limits
         fractionToLimit = 0.99;
         offset = new Vector2D(randomVector);
         heightOffset = 0.0;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, true);


         // test right on distance limits
         fractionToLimit = 1.0 - 1e-4;
         offset = new Vector2D(randomVector);
         heightOffset = 0.0;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, true);

         // test just over distance limits
         fractionToLimit = 1.01;
         offset = new Vector2D(randomVector);
         heightOffset = 0.0;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, false);

         // test well over distance limits
         fractionToLimit = 1.1;
         offset = new Vector2D(randomVector);
         heightOffset = 0.0;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, false);


         // test almost too high
         fractionToLimit = 0.99;
         offset = new Vector2D(randomVector);
         heightOffset = fractionToLimit * heightLimit;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, true);

         // test almost too low
         fractionToLimit = 0.99;
         offset = new Vector2D(randomVector);
         heightOffset = -fractionToLimit * heightLimit;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, true);

         // test too high
         fractionToLimit = 1.01;
         offset = new Vector2D(randomVector);
         heightOffset = fractionToLimit * heightLimit;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, false);

         // test too low
         fractionToLimit = 1.01;
         offset = new Vector2D(randomVector);
         heightOffset = -fractionToLimit * heightLimit;

         offset.scale(fractionToLimit * maxDistance);

         runTest(interRegionConnectionFilter, sourcePoint, heightOffset, offset, false);
      }

   }

   private static void runTest(InterRegionConnectionFilter interRegionFilter,
                               ConnectionPoint3D source, double heightOffset, Vector2DReadOnly offset,
                               boolean shouldBeValid)
   {
      Point3D targetPoint = new Point3D(source.getX(), source.getY(), source.getZ());

      targetPoint.addX(offset.getX());
      targetPoint.addY(offset.getY());
      targetPoint.addZ(heightOffset);

      assertEquals(shouldBeValid, interRegionFilter.isConnectionValid(source, new ConnectionPoint3D(targetPoint, -1)));
   }

   @Test
   public void testFilterRandomConnections()
   {
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      double interRegionDistance = parameters.getMaxInterRegionConnectionLength();
      double connectionDistance = parameters.getNavigableExtrusionDistance();

      InterRegionConnectionFilter interRegionFilter = parameters.getInterRegionConnectionFilter();

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
         assertTrue(message, interRegionFilter.isConnectionValid(source, target));
      }
   }
}
