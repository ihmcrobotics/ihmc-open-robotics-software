package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   @Disabled
   @Test
   public void testWiggleIntoHullBug1()
   {
      ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
      polygonToWiggle.addVertex(-0.272, -0.013);
      polygonToWiggle.addVertex(-0.057, -0.028);
      polygonToWiggle.addVertex(-0.056, -0.088);
      polygonToWiggle.addVertex(-0.271, -0.108);
      polygonToWiggle.update();

      ConvexPolygon2D polygonToWiggleInto = new ConvexPolygon2D();
      polygonToWiggleInto.addVertex(-0.20, 0.20);
      polygonToWiggleInto.addVertex(0.20, 0.20);
      polygonToWiggleInto.addVertex(0.20, -0.20);
      polygonToWiggleInto.addVertex(-0.20, -0.20);
      polygonToWiggleInto.update();

      ConvexStepConstraintOptimizer optimizer = new ConvexStepConstraintOptimizer(new YoRegistry("test"));
      ConstraintOptimizerParameters parameters = new ConstraintOptimizerParameters();

      RigidBodyTransformReadOnly transform = optimizer.findConstraintTransform(polygonToWiggle, polygonToWiggleInto, parameters);

      // check to make sure the original polgyon is outside
      assertFalse(polygonToWiggle.getVertexBufferView().stream().allMatch(polygonToWiggleInto::isPointInside));

      ConvexPolygon2D wiggledPolygon = new ConvexPolygon2D(polygonToWiggle);
      wiggledPolygon.applyTransform(transform);

      // check the transform
      Vector3D expectedTranslation = new Vector3D(0.071, 0.0, 0.0);
      EuclidCoreTestTools.assertEquals(expectedTranslation, transform.getTranslation(), 1e-4);

      // check to make sure the transformed polgyon is inside
      assertTrue("Not all points are inside", polygonToWiggle.getVertexBufferView().stream().allMatch(polygonToWiggleInto::isPointInside));
   }

   @Disabled
   @Test
   public void testWiggleIntoHullBug2()
   {
      ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
      polygonToWiggle.addVertex(-0.2408766530535793, 0.12750211571302827);
      polygonToWiggle.addVertex(-0.025876631931942617, 0.11000237520935868);
      polygonToWiggle.addVertex(-0.0258765595143656, 0.0500023752094024);
      polygonToWiggle.addVertex(-0.24087653839241568, 0.032502115713097454);
      polygonToWiggle.update();

      ConvexPolygon2D polygonToWiggleInto = new ConvexPolygon2D();
      polygonToWiggleInto.addVertex(-0.20, 0.20);
      polygonToWiggleInto.addVertex(0.20, 0.20);
      polygonToWiggleInto.addVertex(0.20, -0.20);
      polygonToWiggleInto.addVertex(-0.20, -0.20);
      polygonToWiggleInto.update();

      ConvexStepConstraintOptimizer optimizer = new ConvexStepConstraintOptimizer(new YoRegistry("test"));
      ConstraintOptimizerParameters parameters = new ConstraintOptimizerParameters();

      RigidBodyTransformReadOnly transform = optimizer.findConstraintTransform(polygonToWiggle, polygonToWiggleInto, parameters);

      // check to make sure the original polgyon is outside
      assertFalse(polygonToWiggle.getVertexBufferView().stream().allMatch(polygonToWiggleInto::isPointInside));

      ConvexPolygon2D wiggledPolygon = new ConvexPolygon2D(polygonToWiggle);
      wiggledPolygon.applyTransform(transform);

      Vector3D expectedTranslation = new Vector3D(0.071, 0.0, 0.0);
      EuclidCoreTestTools.assertEquals(expectedTranslation, transform.getTranslation(), 1e-4);

      // check to make sure the transformed polgyon is inside
      assertTrue(polygonToWiggle.getVertexBufferView().stream().allMatch(polygonToWiggleInto::isPointInside));
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
