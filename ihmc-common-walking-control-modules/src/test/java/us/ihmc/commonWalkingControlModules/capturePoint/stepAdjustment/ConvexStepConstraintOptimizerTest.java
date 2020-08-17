package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

import static us.ihmc.robotics.Assert.*;

public class ConvexStepConstraintOptimizerTest
{
   @Test
   public void testProjectionIntoPlanarRegionHull()
   {
      ArrayList<ConvexPolygon2D> planes = new ArrayList<>();
      ConvexPolygon2D plane1 = new ConvexPolygon2D();
      plane1.addVertex(0.0, 0.0);
      plane1.addVertex(0.5, 0.0);
      plane1.addVertex(0.0, 0.5);
      plane1.addVertex(0.5, 0.5);
      plane1.update();
      planes.add(plane1);

      ConvexPolygon2D plane2 = new ConvexPolygon2D();
      plane2.addVertex(-0.6, 0.0);
      plane2.addVertex(-0.1, 0.0);
      plane2.addVertex(-0.6, 0.5);
      plane2.addVertex(-0.1, 0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2D initialFoot = createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.getTranslation().set(-0.05, 0.05, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);


      ConstraintOptimizerParameters parameters = new ConstraintOptimizerParameters();
      ConvexStepConstraintOptimizer stepConstraintOptimizer = new ConvexStepConstraintOptimizer(new YoRegistry("test"));
      RigidBodyTransformReadOnly wiggleTransform = stepConstraintOptimizer.findConstraintTransform(initialFoot, region.getConvexHull(), parameters);

      assertFalse(wiggleTransform == null);

      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);

      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, region.getConvexHull()));
      foot.applyTransform(wiggleTransform, false);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, region.getConvexHull()));

      // Now, let's translation it a lot, and make sure the constraints on max translation prevent it from moving too much
      initialFootTransform.setToZero();
      initialFootTransform.getTranslation().set(1.0, 1.0, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      parameters.setMaxX(0.05);
      parameters.setMaxY(0.05);

      foot.set(initialFoot);
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, region.getConvexHull()));
      wiggleTransform = stepConstraintOptimizer.findConstraintTransform(initialFoot, region.getConvexHull(), parameters);

      assertEquals(-0.05, wiggleTransform.getTranslationX(), 1e-6);
      assertEquals(-0.05, wiggleTransform.getTranslationY(), 1e-6);


      foot.applyTransform(wiggleTransform, false);
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, region.getConvexHull()));

      parameters.setConstrainMaxAdjustment(false);
      wiggleTransform = stepConstraintOptimizer.findConstraintTransform(initialFoot, region.getConvexHull(), parameters);
      foot.set(initialFoot);
      foot.applyTransform(wiggleTransform, false);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, region.getConvexHull()));


      double distanceInside = 0.05;
      parameters.setDesiredDistanceInside(distanceInside);
      assertFalse(isPolygonDistanceInside(foot, distanceInside, 1e-5, region.getConvexHull()));
      wiggleTransform = stepConstraintOptimizer.findConstraintTransform(foot, region.getConvexHull(), parameters);
      foot.applyTransform(wiggleTransform, false);
      assertTrue(isPolygonDistanceInside(foot, distanceInside, 1e-5, region.getConvexHull()));

   }

   private static boolean isPolygonDistanceInside(ConvexPolygon2DReadOnly polygonToTest, double distance, double epsilon, ConvexPolygon2DReadOnly polygon)
   {
      for (int i = 0; i < polygonToTest.getNumberOfVertices(); i++)
      {
         double distanceInside = polygon.signedDistance(polygonToTest.getVertex(i));
         if (distanceInside > -distance + epsilon)
            return false;
      }

      return true;
   }

   public static ConvexPolygon2D createDefaultFootPolygon()
   {
      return createFootPolygon(0.2, 0.1, 0.1);
   }

   public static ConvexPolygon2D createFootPolygon(double footLength, double heelWidth, double toeWidth)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, heelWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -heelWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }
}
