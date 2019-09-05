package us.ihmc.footstepPlanning.polygonWiggling;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class PolygonWigglingTest
{
   private static final boolean visualize = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());

   private final static double epsilon = 0.00001;

   @Test
   public void testSimpleProjection()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testProjectionBestEffort()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = 0.5; // Not possible, should do the best it can.
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      // A deltaInside of 0.1 is possible in this case so the wiggler should have done at least that.
      checkThatWiggledInsideBestEffort(plane, 0.1, foot);
   }

   private void checkThatWiggledInsideBestEffort(ConvexPolygon2D plane, double minDelta, ConvexPolygon2D foot)
   {
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));

      for (int i = 0; i < foot.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = foot.getVertex(i);
         double distance = Math.abs(plane.signedDistance(vertex));
         assertTrue(distance > minDelta);
      }
   }

   @Test
   public void testSimpleProjectionWithDeltaInside()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = 0.06;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      checkThatWiggledInsideJustTheRightAmount(plane, wiggleParameters, foot);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));

   }

   private void checkThatWiggledInsideJustTheRightAmount(ConvexPolygon2D plane, WiggleParameters wiggleParameters, ConvexPolygon2D foot)
   {
      double largestDistance = Double.NEGATIVE_INFINITY;

      for (int i=0; i<foot.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = foot.getVertex(i);
         double signedDistance = plane.signedDistance(vertex);

         if (signedDistance > largestDistance)
         {
            largestDistance = signedDistance;
         }
      }
      assertTrue(largestDistance < -wiggleParameters.deltaInside);
      assertFalse(largestDistance < -wiggleParameters.deltaInside - 0.003);
   }

   @Test
   public void testSimpleProjectionWithWiggleLimits()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.02;
      parameters.maxX = 0.02;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      double initialDistance = initialFoot.getCentroid().distance(plane.getCentroid());
      double finalDistance = foot.getCentroid().distance(plane.getCentroid());
      assertTrue(initialDistance > finalDistance);
   }

   @Test
   public void testProjectionThatRequiredRotation()
   {
      ConvexPolygon2D plane = PlannerTools.createDefaultFootPolygon();
      plane.scale(1.1);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-13.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, new WiggleParameters());

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testImpossibleCases()
   {
      ConvexPolygon2D plane = PlannerTools.createDefaultFootPolygon();
      plane.scale(0.9);
      addPolygonToArtifacts("Plane", plane, Color.BLACK);

      Random random = new Random(382848284829L);
      double yawLimit = Math.toRadians(15.0);
      for (int i = 0; i < 1000; i ++)
      {
         ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
         RigidBodyTransform initialFootTransform = new RigidBodyTransform();
         double x = 5.0 * (random.nextDouble() - 0.5);
         double y = 5.0 * (random.nextDouble() - 0.5);
         double theta = 2.0 * (random.nextDouble() - 0.5) * yawLimit;
         initialFootTransform.setRotationYawAndZeroTranslation(theta);
         initialFootTransform.setTranslation(x, y, 0.0);
         initialFoot.applyTransform(initialFootTransform, false);

         WiggleParameters parameters = new WiggleParameters();
         parameters.minX = -5.0;
         parameters.maxX = 5.0;
         parameters.minY = -5.0;
         parameters.maxY = 5.0;
         parameters.minYaw = -5.0;
         parameters.maxYaw = 5.0;
         ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);
         assertTrue(ConvexPolygon2dCalculator.isPolygonInside(plane, 1.0e-5, foot));

         if (visualize)
         {
            addPolygonToArtifacts("InitialFoot" + i, initialFoot, Color.RED);
            addPolygonToArtifacts("Foot" + i, foot, Color.BLUE);
         }
      }

      if (visualize)
      {
         showPlotterAndSleep(artifacts);
      }

   }

   @Test
   public void testProjectionOnlyTranslation()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(-0.2, 0.25, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, new WiggleParameters());

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testProjectionTranslationLimits()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(-0.2, 0.25, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 0.1;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      double initialDistance = initialFoot.getCentroid().distance(plane.getCentroid());
      double finalDistance = foot.getCentroid().distance(plane.getCentroid());
      assertTrue(initialDistance > finalDistance);
   }

   @Test
   public void testProjectionTranslationLimitX1()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.0, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 1.0e-15;
      parameters.maxY = 1.0;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      Point2DReadOnly footCentroid = foot.getCentroid();
      Point2DReadOnly initialFootCentroid = initialFoot.getCentroid();

      System.out.println("footCentroid: " + footCentroid);
      System.out.println("initialFootCentroid: " + initialFootCentroid);

      assertTrue(footCentroid.getX() - initialFootCentroid.getX() <= parameters.maxX);
      assertTrue(footCentroid.getX() - initialFootCentroid.getX() >= parameters.minX);
      assertTrue(footCentroid.getY() - initialFootCentroid.getY() <= parameters.maxY);
      assertTrue(footCentroid.getY() - initialFootCentroid.getY() >= parameters.minY);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testProjectionTranslationLimitX2()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(1.0, 1.5, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.0;
      parameters.minY = -1.0;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() <= parameters.maxX);
      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() >= parameters.minX);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() <= parameters.maxY);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() >= parameters.minY);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testProjectionTranslationLimitY1()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.0, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 1.0;
      parameters.maxY = 0.05;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() <= parameters.maxX);
      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() >= parameters.minX);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() <= parameters.maxY);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() >= parameters.minY);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testProjectionTranslationLimitY2()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(1.0, 1.5, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = -1.0;
      parameters.minY = 0.0;
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() <= parameters.maxX);
      assertTrue(foot.getCentroid().getX() - initialFoot.getCentroid().getX() >= parameters.minX);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() <= parameters.maxY);
      assertTrue(foot.getCentroid().getY() - initialFoot.getCentroid().getY() >= parameters.minY);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @Test
   public void testKnownResult()
   {
      // this is a regression test - this will check if the expected result is produced.
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.05, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.rotationWeight = 0.2;
      parameters.maxX = 0.5;
      parameters.minX = -0.5;
      parameters.maxY = 0.5;
      parameters.minY = -0.5;
      parameters.maxYaw = Math.toRadians(15.0);
      parameters.minYaw = -Math.toRadians(15.0);
      ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));

      // expected:
      ArrayList<Point2D> expected = new ArrayList<>();
      expected.add(new Point2D(0.6946910259467516, 0.3057219822306081));
      expected.add(new Point2D(0.7021375429674444, 0.405444343736243));
      expected.add(new Point2D(0.901582265978714, 0.39055130969485763));
      expected.add(new Point2D(0.8941357489580215, 0.2908289481892227));

      for (int i = 0; i < foot.getNumberOfVertices(); i++)
      {
         if (!foot.getVertex(i).epsilonEquals(expected.get(i), 1.0E-5))
         {
            fail("Failed at vertex index: " + i + ", expected: " + expected + ", was: " + foot);
         }
      }
   }

   @Test
   public void testCompexProjectionArea()
   {
      ConvexPolygon2D plane = new ConvexPolygon2D();
      plane.addVertex(0.5, -0.5);
      plane.addVertex(-0.8, 0.5);
      plane.addVertex(0.5, 1.5);
      plane.addVertex(0.75, 0.5);
      plane.update();
      addPolygonToArtifacts("Plane", plane, Color.BLACK);

      double yawLimit = Math.toRadians(15.0);
      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.maxYaw = yawLimit;
      wiggleParameters.minYaw = -yawLimit;
      wiggleParameters.maxX = 10.0;
      wiggleParameters.minX = -10.0;
      wiggleParameters.maxY = 10.0;
      wiggleParameters.minY = -10.0;
      Random random = new Random(482787427467L);

      for (int i = 0; i < 1000; i++)
      {
         ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
         if (random.nextBoolean())
         {
            initialFoot.removeVertex(random.nextInt(4));
            initialFoot.update();
         }

         RigidBodyTransform initialFootTransform = new RigidBodyTransform();
         double x = 5.0 * (random.nextDouble() - 0.5);
         double y = 5.0 * (random.nextDouble() - 0.5);
         double theta = 2.0 * (random.nextDouble() - 0.5) * yawLimit;
         initialFootTransform.setRotationYawAndZeroTranslation(theta);
         initialFootTransform.setTranslation(x, y, 0.0);
         initialFoot.applyTransform(initialFootTransform, false);

         ConvexPolygon2D foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);
         assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
         if (ConvexPolygon2dCalculator.isPolygonInside(initialFoot, 1.0e-5, plane))
            assertTrue(initialFoot.epsilonEquals(foot, 1.0e-5));

         if (visualize)
         {
            addPolygonToArtifacts("InitialFoot" + i, initialFoot, Color.RED);
            addPolygonToArtifacts("Foot" + i, foot, Color.BLUE);
         }
      }

      if (visualize)
      {
         showPlotterAndSleep(artifacts);
      }
   }

   @Test
   public void testProjectionIntoPlanarRegion1()
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
      ConvexPolygon2D plane3 = new ConvexPolygon2D();
      plane3.addVertex(-0.25, 0.0);
      plane3.addVertex(0.25, 0.0);
      plane3.addVertex(-0.25, -0.5);
      plane3.addVertex(0.25, -0.5);
      plane3.update();
      planes.add(plane3);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);
      foot.applyTransform(wiggleTransfrom, false);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane2));
   }

   @Test
   public void testProjectionIntoPlanarRegion2()
   {
      ArrayList<ConvexPolygon2D> planes = new ArrayList<>();
      ConvexPolygon2D plane1 = new ConvexPolygon2D();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2D plane2 = new ConvexPolygon2D();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);
      foot.applyTransform(wiggleTransfrom, false);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane1));
   }

   @Test
   public void testProjectionIntoPlanarRegionNoOverlap()
   {
      ArrayList<ConvexPolygon2D> planes = new ArrayList<>();
      ConvexPolygon2D plane1 = new ConvexPolygon2D();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2D plane2 = new ConvexPolygon2D();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(0.1, 0.1, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, new WiggleParameters());
      assertTrue(wiggleTransfrom == null);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         showPlotterAndSleep(artifacts);
      }
   }

   @Test
   public void testProjectionIntoPlanarRegionInvalidLimits()
   {
      ArrayList<ConvexPolygon2D> planes = new ArrayList<>();
      ConvexPolygon2D plane1 = new ConvexPolygon2D();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2D plane2 = new ConvexPolygon2D();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.0;
      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, parameters);
      assertTrue(wiggleTransfrom != null);
      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);
      foot.applyTransform(wiggleTransfrom, false);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }
   }

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

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.05, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2D foot = new ConvexPolygon2D(initialFoot);
      foot.applyTransform(wiggleTransfrom, false);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      ConvexPolygon2D hullOfRegion = new ConvexPolygon2D(plane1, plane2);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, hullOfRegion));
   }

   @Test
   public void testConvexConstraintOfPoint()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D point = new Point2D();
      Random random = new Random(234235L);

      for (int iter = 0; iter < 1000; iter++)
      {
         point.set(random.nextDouble(), random.nextDouble());
         polygon.clear();
         polygon.addVertex(point);
         polygon.update();
         PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.0);

         // test actual point satisfies constraint
         x.set(0, 0, point.getX());
         x.set(1, 0, point.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0));

         // test if slightly both greater the actual point satisfies constraint
         double offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         boolean allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x slightly less the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if y slightly less the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX());
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x slightly both the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if y slightly both the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX());
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if both slightly less the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x slightly greater y slightly less the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x slightly less y slightly greater the actual point satisfies constraint
         offset = 0.05 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if a lot off the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x a lot less the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if y a lot less the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX());
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x a lot both the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if y a lot both the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX());
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if both a lot less the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x a lot greater y a lot less the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() + offset);
         x.set(1, 0, point.getY() - offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);

         // test if x a lot less y a lot greater the actual point satisfies constraint
         offset = 1.5 * random.nextDouble();
         x.set(0, 0, point.getX() - offset);
         x.set(1, 0, point.getY() + offset);

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0);
         assertFalse(allLessThan);
      }
   }

   // TODO: 05/10/2017 moved to development
   @Disabled
   @Test
   public void testConvexConstraintOfLine()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D point1 = new Point2D();
      Point2D point2 = new Point2D();
      Point2D point3 = new Point2D();

      Random random = new Random(-8133358861874482661L);

      for (int iters= 0; iters < 1000; iters++)
      {
         point1.set(random.nextDouble(), random.nextDouble());
         point2.set(random.nextDouble(), random.nextDouble());

         polygon.clear();
         polygon.addVertex(point1);
         polygon.addVertex(point2);
         polygon.update();

         PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.0);

         solution.reshape(b.getNumRows(), b.getNumCols());

         // test point1 satisfies constraint
         x.set(0, 0, point1.getX());
         x.set(1, 0, point1.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test point2 satisfies constraint
         x.set(0, 0, point2.getX());
         x.set(1, 0, point2.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test midpoint satisfies constraint
         point3.interpolate(point1, point2, 0.5);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test slightly past point 1 satisfies constraint
         point3.interpolate(point1, point2, 0.1);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test slightly before point 2 satisfies constraint
         point3.interpolate(point1, point2, 0.9);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test random interior point satisfies constraint
         point3.interpolate(point1, point2, random.nextDouble());
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test slightly less than point 1 fails constraint
         point3.interpolate(point1, point2, -0.1);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         boolean allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test slightly less than point 2 fails constraint
         point3.interpolate(point1, point2, 1.1);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test slightly way less than point 1 fails constraint
         point3.interpolate(point1, point2, -1.0);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test slightly way past point 2 fails constraint
         point3.interpolate(point1, point2, 2.0);
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test randomly less than point 1 fails constraint
         point3.interpolate(point1, point2, -random.nextDouble());
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test randomly past point 2 fails constraint
         point3.interpolate(point1, point2, 1.0 + random.nextDouble());
         x.set(0, 0, point3.getX());
         x.set(1, 0, point3.getY());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test perpendicular point1 fails constraint
         x.set(0, 0, point1.getY());
         x.set(1, 0, point1.getX());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test perpendicular point2 fails constraint
         x.set(0, 0, point2.getY());
         x.set(1, 0, point2.getX());

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);

         // test slightly off midpoint fails constraint
         point3.interpolate(point1, point2, 0.5);
         x.set(0, 0, point3.getX() + (0.5 - random.nextDouble()));
         x.set(1, 0, point3.getY() + (0.5 - random.nextDouble()));

         CommonOps.mult(A, x, solution);
         allLessThan = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allLessThan);
      }
   }

   @Test
   public void testConvexConstraintOfSimpleLine()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D point1 = new Point2D();
      Point2D point2 = new Point2D();
      Point2D point3 = new Point2D();

      point1.set(0.0, 0.0);
      point2.set(1.0, 0.0);
      polygon.addVertex(point1);
      polygon.addVertex(point2);
      polygon.update();
      PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.0);

      solution.reshape(b.getNumRows(), b.getNumCols());

      // test point1 satisfies constraint
      x.set(0, 0, point1.getX());
      x.set(1, 0, point1.getY());

      CommonOps.mult(A, x, solution);
      for (int i = 0; i < solution.getNumRows(); i++)
         assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

      // test point 2 satisfies constraint
      x.set(0, 0, point2.getX());
      x.set(1, 0, point2.getY());

      CommonOps.mult(A, x, solution);
      for (int i = 0; i < solution.getNumRows(); i++)
         assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

      // test midpoint satisfies constraint
      point3.interpolate(point1, point2, 0.5);
      x.set(0, 0, point3.getX());
      x.set(1, 0, point3.getY());

      CommonOps.mult(A, x, solution);
      for (int i = 0; i < solution.getNumRows(); i++)
         assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);



      // on line but before segment
      x.set(0, 0, -1.0);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      boolean allLessThan = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allLessThan);




      // on line but past segment
      x.set(0, 0, 2.0);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allLessThan = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allLessThan &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allLessThan);
   }

   @Test
   public void testConvexConstraintOfDeltaInsideLine()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D point1 = new Point2D();
      Point2D point2 = new Point2D();
      Point2D point3 = new Point2D();

      point1.set(0.0, 0.0);
      point2.set(1.0, 0.0);
      polygon.addVertex(point1);
      polygon.addVertex(point2);
      polygon.update();
      PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.1);

      solution.reshape(b.getNumRows(), b.getNumCols());

      // test point1 does not satisfy constraint
      x.set(0, 0, point1.getX());
      x.set(1, 0, point1.getY());

      CommonOps.mult(A, x, solution);
      boolean allPointsInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allPointsInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allPointsInside);

      // test point 2 does not satisfy constraint
      x.set(0, 0, point2.getX());
      x.set(1, 0, point2.getY());

      CommonOps.mult(A, x, solution);
      allPointsInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allPointsInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allPointsInside);

      // test midpoint satisfies constraint
      point3.interpolate(point1, point2, 0.5);
      x.set(0, 0, point3.getX());
      x.set(1, 0, point3.getY());

      CommonOps.mult(A, x, solution);
      for (int i = 0; i < solution.getNumRows(); i++)
         assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);
   }

   // TODO: 05/03/2017 moved to development
   @Disabled
   @Test
   public void testConvexConstraintOfQuadrangle()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D firstQuadrantPoint = new Point2D();
      Point2D secondQuadrantPoint = new Point2D();
      Point2D thirdQuadrantPoint = new Point2D();
      Point2D fourthQuadrantPoint = new Point2D();

      Random random = new Random(3167568458681184485L);

      for (int iters = 0; iters < 1000; iters++)
      {

         firstQuadrantPoint.set(1.0 + random.nextDouble(), 1.0 + random.nextDouble());
         secondQuadrantPoint.set(1.0 + random.nextDouble(), -1.0 - random.nextDouble());
         thirdQuadrantPoint.set(-1.0 - random.nextDouble(), -1.0 - random.nextDouble());
         fourthQuadrantPoint.set(-1.0 - random.nextDouble(), 1.0 + random.nextDouble());

         polygon.clear();
         polygon.addVertex(firstQuadrantPoint);
         polygon.addVertex(secondQuadrantPoint);
         polygon.addVertex(thirdQuadrantPoint);
         polygon.addVertex(fourthQuadrantPoint);
         polygon.update();

         PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.0);

         solution.reshape(b.getNumRows(), b.getNumCols());

         // test firstQuadrantPoint satisfies constraint
         x.set(0, 0, firstQuadrantPoint.getX());
         x.set(1, 0, firstQuadrantPoint.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test secondQuadrantPoint satisfies constraint
         x.set(0, 0, secondQuadrantPoint.getX());
         x.set(1, 0, secondQuadrantPoint.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test thirdQuadrantPoint satisfies constraint
         x.set(0, 0, thirdQuadrantPoint.getX());
         x.set(1, 0, thirdQuadrantPoint.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test fourthQuadrantPoint satisfies constraint
         x.set(0, 0, fourthQuadrantPoint.getX());
         x.set(1, 0, fourthQuadrantPoint.getY());

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test firstQuadrantPoint slightly inside satisfies constraint
         double slightOffset = 0.1 * random.nextDouble();
         x.set(0, 0, firstQuadrantPoint.getX() - slightOffset);
         x.set(1, 0, firstQuadrantPoint.getY() - slightOffset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test secondQuadrantPoint satisfies constraint
         x.set(0, 0, secondQuadrantPoint.getX() - slightOffset);
         x.set(1, 0, secondQuadrantPoint.getY() + slightOffset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test thirdQuadrantPoint satisfies constraint
         x.set(0, 0, thirdQuadrantPoint.getX() + slightOffset);
         x.set(1, 0, thirdQuadrantPoint.getY() + slightOffset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test fourthQuadrantPoint satisfies constraint
         x.set(0, 0, fourthQuadrantPoint.getX() + slightOffset);
         x.set(1, 0, fourthQuadrantPoint.getY() - slightOffset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test firstQuadrantPoint big inside satisfies constraint
         double offset = random.nextDouble();
         x.set(0, 0, firstQuadrantPoint.getX() - offset);
         x.set(1, 0, firstQuadrantPoint.getY() - offset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test secondQuadrantPoint satisfies constraint
         x.set(0, 0, secondQuadrantPoint.getX() - offset);
         x.set(1, 0, secondQuadrantPoint.getY() + offset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test thirdQuadrantPoint satisfies constraint
         x.set(0, 0, thirdQuadrantPoint.getX() + offset);
         x.set(1, 0, thirdQuadrantPoint.getY() + offset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test fourthQuadrantPoint satisfies constraint
         x.set(0, 0, fourthQuadrantPoint.getX() + offset);
         x.set(1, 0, fourthQuadrantPoint.getY() - offset);

         CommonOps.mult(A, x, solution);
         for (int i = 0; i < solution.getNumRows(); i++)
            assertTrue(solution.get(i, 0) <= b.get(i, 0) + epsilon);

         // test firstQuadrantPoint slight outside violates constraint
         x.set(0, 0, firstQuadrantPoint.getX() + slightOffset);
         x.set(1, 0, firstQuadrantPoint.getY() + slightOffset);

         CommonOps.mult(A, x, solution);
         boolean allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test secondQuadrantPoint satisfies constraint
         x.set(0, 0, secondQuadrantPoint.getX() + slightOffset);
         x.set(1, 0, secondQuadrantPoint.getY() - slightOffset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test thirdQuadrantPoint satisfies constraint
         x.set(0, 0, thirdQuadrantPoint.getX() - slightOffset);
         x.set(1, 0, thirdQuadrantPoint.getY() - slightOffset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test fourthQuadrantPoint satisfies constraint
         x.set(0, 0, fourthQuadrantPoint.getX() - slightOffset);
         x.set(1, 0, fourthQuadrantPoint.getY() + slightOffset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test firstQuadrantPoint big outside violates constraint
         x.set(0, 0, firstQuadrantPoint.getX() + offset);
         x.set(1, 0, firstQuadrantPoint.getY() + offset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test secondQuadrantPoint satisfies constraint
         x.set(0, 0, secondQuadrantPoint.getX() + offset);
         x.set(1, 0, secondQuadrantPoint.getY() - offset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test thirdQuadrantPoint satisfies constraint
         x.set(0, 0, thirdQuadrantPoint.getX() - offset);
         x.set(1, 0, thirdQuadrantPoint.getY() - offset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);

         // test fourthQuadrantPoint satisfies constraint
         x.set(0, 0, fourthQuadrantPoint.getX() - offset);
         x.set(1, 0, fourthQuadrantPoint.getY() + offset);

         CommonOps.mult(A, x, solution);
         allConditions = true;
         for (int i = 0; i < solution.getNumRows(); i++)
            allConditions &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
         assertFalse(allConditions);
      }
   }

   @Test
   public void testConvexConstraintOfQuadrangleDeltaInside()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D firstQuadrantPoint = new Point2D();
      Point2D secondQuadrantPoint = new Point2D();
      Point2D thirdQuadrantPoint = new Point2D();
      Point2D fourthQuadrantPoint = new Point2D();

      Random random = new Random(234235L);

      firstQuadrantPoint.set(1.0 + random.nextDouble(), 1.0 + random.nextDouble());
      secondQuadrantPoint.set(1.0 + random.nextDouble(), -1.0 - random.nextDouble());
      thirdQuadrantPoint.set(-1.0 - random.nextDouble(), -1.0 - random.nextDouble());
      fourthQuadrantPoint.set(-1.0 - random.nextDouble(), 1.0 + random.nextDouble());

      polygon.clear();
      polygon.addVertex(firstQuadrantPoint);
      polygon.addVertex(secondQuadrantPoint);
      polygon.addVertex(thirdQuadrantPoint);
      polygon.addVertex(fourthQuadrantPoint);
      polygon.update();

      PolygonWiggler.convertToInequalityConstraints(polygon, A, b, 0.1 * random.nextDouble());

      solution.reshape(b.getNumRows(), b.getNumCols());

      // test firstQuadrantPoint violates constraint
      x.set(0, 0, firstQuadrantPoint.getX());
      x.set(1, 0, firstQuadrantPoint.getY());

      CommonOps.mult(A, x, solution);
      boolean allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test secondQuadrantPoint violates constraint
      x.set(0, 0, secondQuadrantPoint.getX());
      x.set(1, 0, secondQuadrantPoint.getY());

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test thirdQuadrantPoint violates constraint
      x.set(0, 0, thirdQuadrantPoint.getX());
      x.set(1, 0, thirdQuadrantPoint.getY());

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test fourthQuadrantPoint violates constraint
      x.set(0, 0, fourthQuadrantPoint.getX());
      x.set(1, 0, fourthQuadrantPoint.getY());

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);
   }

   @Test
   public void testConstraintOfSquarePolygonInSquarePolygon()
   {
      DenseMatrix64F A = new DenseMatrix64F(4, 2);
      DenseMatrix64F b = new DenseMatrix64F(4);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(4, 1);

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();

      exteriorPolygon.addVertex(new Point2D(1.0, 1.0));
      exteriorPolygon.addVertex(new Point2D(1.0, -1.0));
      exteriorPolygon.addVertex(new Point2D(-1.0, -1.0));
      exteriorPolygon.addVertex(new Point2D(-1.0, 1.0));
      exteriorPolygon.update();

      interiorPolygon.addVertex(new Point2D(0.5, 0.5));
      interiorPolygon.addVertex(new Point2D(0.5, -0.5));
      interiorPolygon.addVertex(new Point2D(-0.5, -0.5));
      interiorPolygon.addVertex(new Point2D(-0.5, 0.5));
      interiorPolygon.update();

      PolygonWiggler.constrainPolygonInsideOtherPolygon(exteriorPolygon, interiorPolygon, A, b, 0.0);

      // test centered
      x.set(0, 0, 0.0);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      boolean allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on left edge
      x.set(0, 0, -0.5);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on right edge
      x.set(0, 0, 0.5);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on top edge
      x.set(0, 0, 0.0);
      x.set(1, 0, 0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on bottom edge
      x.set(0, 0, 0.0);
      x.set(1, 0, -0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on top left corner
      x.set(0, 0, -0.5);
      x.set(1, 0, 0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on top right corner
      x.set(0, 0, 0.5);
      x.set(1, 0, 0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on bottom right corner
      x.set(0, 0, 0.5);
      x.set(1, 0, -0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test on bottom left corner
      x.set(0, 0, -0.5);
      x.set(1, 0, -0.5);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test outside left edge
      x.set(0, 0, 0.0);
      x.set(1, 0, -0.6);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test outside right edge
      x.set(0, 0, 0.0);
      x.set(1, 0, 0.6);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test outside top edge
      x.set(0, 0, 0.6);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test outside bottom edge
      x.set(0, 0, -0.6);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test outside top left corner
      x.set(0, 0, 0.6);
      x.set(1, 0, -0.6);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);
   }

   @Test
   public void testConstraintOfSquarePolygonInPentagon()
   {
      DenseMatrix64F A = new DenseMatrix64F(5, 2);
      DenseMatrix64F b = new DenseMatrix64F(5);

      DenseMatrix64F x = new DenseMatrix64F(2, 1);
      DenseMatrix64F solution = new DenseMatrix64F(5, 1);

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();

      exteriorPolygon.addVertex(new Point2D(-1.0, -0.6));
      exteriorPolygon.addVertex(new Point2D(-1.0, 0.6));
      exteriorPolygon.addVertex(new Point2D(0.0, -1.0));
      exteriorPolygon.addVertex(new Point2D(0.0, 1.0));
      exteriorPolygon.addVertex(new Point2D(1.0, 0.0));
      exteriorPolygon.update();

      interiorPolygon.addVertex(new Point2D(0.5, 0.5));
      interiorPolygon.addVertex(new Point2D(0.5, -0.5));
      interiorPolygon.addVertex(new Point2D(-0.5, -0.5));
      interiorPolygon.addVertex(new Point2D(-0.5, 0.5));
      interiorPolygon.update();

      PolygonWiggler.constrainPolygonInsideOtherPolygon(exteriorPolygon, interiorPolygon, A, b, 0.0);

      // test centered
      x.set(0, 0, 0.0);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      boolean allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test a little past as far up as possible
      x.set(0, 0, 0.1);
      x.set(1, 0, 0.0);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test a little left
      x.set(0, 0, 0.0);
      x.set(1, 0, -0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test a little right
      x.set(0, 0, 0.0);
      x.set(1, 0, 0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test at bottom left
      x.set(0, 0, -0.5);
      x.set(1, 0, -0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test a little left of bottom left
      x.set(0, 0, -0.5);
      x.set(1, 0, -0.2);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test a little below bottom left
      x.set(0, 0, -0.6);
      x.set(1, 0, -0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test at bottom right
      x.set(0, 0, -0.5);
      x.set(1, 0, 0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertTrue(allInside);

      // test a little right of bottom right
      x.set(0, 0, -0.5);
      x.set(1, 0, 0.2);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);

      // test a little below bottom right
      x.set(0, 0, -0.6);
      x.set(1, 0, 0.1);

      CommonOps.mult(A, x, solution);
      allInside = true;
      for (int i = 0; i < solution.getNumRows(); i++)
         allInside &= solution.get(i, 0) <= b.get(i, 0) + epsilon;
      assertFalse(allInside);
   }

   private void addPolygonToArtifacts(String name, ConvexPolygon2D polygon, Color color)
   {
      YoFrameConvexPolygon2D yoPlanePolygon = new YoFrameConvexPolygon2D(name + "Polygon", worldFrame, 10, registry);
      artifacts.add(new YoArtifactPolygon(name, yoPlanePolygon , color, false));
      yoPlanePolygon.set(polygon);
   }

   private static void showPlotterAndSleep(ArtifactList artifacts)
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(2.0);
      artifacts.setVisible(true);
      JFrame frame = new JFrame("PolygonWigglingTest");
      Dimension preferredSize = new Dimension(600, 600);
      frame.setPreferredSize(preferredSize);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);
      frame.setSize(preferredSize);
      frame.setVisible(true);
      artifacts.addArtifactsToPlotter(plotter);
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PolygonWiggler.class, PolygonWigglingTest.class);
   }
}
