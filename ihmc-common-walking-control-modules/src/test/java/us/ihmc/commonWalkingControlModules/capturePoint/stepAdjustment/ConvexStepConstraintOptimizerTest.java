package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.awt.*;
import java.util.ArrayList;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class ConvexStepConstraintOptimizerTest
{
   @Test
   public void test()
   {}

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

      ConvexStepConstraintOptimizer stepConstraintOptimizer = new ConvexStepConstraintOptimizer();
      RigidBodyTransform wiggleTransfrom = stepConstraintOptimizer.wigglePolygonIntoConvexHullOfRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);
      foot.applyTransform(wiggleTransfrom, false);


      ConvexPolygon2D hullOfRegion = new ConvexPolygon2D(plane1, plane2);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, hullOfRegion));
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
