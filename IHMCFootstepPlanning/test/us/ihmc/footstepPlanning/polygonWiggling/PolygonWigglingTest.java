package us.ihmc.footstepPlanning.polygonWiggling;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PolygonWigglingTest
{
   private static final boolean visualize = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleProjection()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters wiggleParameters = new WiggleParameters();
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleProjectionWithDeltaInside()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = 0.06;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);

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

   private void checkThatWiggledInsideJustTheRightAmount(ConvexPolygon2d plane, WiggleParameters wiggleParameters, ConvexPolygon2d foot)
   {
      double largestDistance = Double.NEGATIVE_INFINITY;

      for (int i=0; i<foot.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = foot.getVertex(i);
         double signedDistance = ConvexPolygon2dCalculator.getSignedDistance(vertex, plane);

         if (signedDistance > largestDistance)
         {
            largestDistance = signedDistance;
         }
      }
      assertTrue(largestDistance < -wiggleParameters.deltaInside);
      assertFalse(largestDistance < -wiggleParameters.deltaInside - 0.003);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleProjectionWithWiggleLimits()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.02;
      parameters.maxX = 0.02;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(foot == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionThatRequiredRotation()
   {
      ConvexPolygon2d plane = PlanningTestTools.createDefaultFootPolygon();
      plane.scale(1.1);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-13.0));
      initialFootTransform.setTranslation(-0.1, -0.3, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, new WiggleParameters());

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testImpossibleCases()
   {
      ConvexPolygon2d plane = PlanningTestTools.createDefaultFootPolygon();
      plane.scale(0.9);
      addPolygonToArtifacts("Plane", plane, Color.BLACK);

      Random random = new Random(382848284829L);
      double yawLimit = Math.toRadians(15.0);
      for (int i = 0; i < 1000; i ++)
      {
         ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
         RigidBodyTransform initialFootTransform = new RigidBodyTransform();
         double x = 5.0 * (random.nextDouble() - 0.5);
         double y = 5.0 * (random.nextDouble() - 0.5);
         double theta = 2.0 * (random.nextDouble() - 0.5) * yawLimit;
         initialFootTransform.setRotationYawAndZeroTranslation(theta);
         initialFootTransform.setTranslation(x, y, 0.0);
         initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

         ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, new WiggleParameters());
         assertTrue(foot == null);

         if (visualize)
         {
            addPolygonToArtifacts("InitialFoot" + i, initialFoot, Color.RED);
         }
      }

      if (visualize)
      {
         showPlotterAndSleep(artifacts);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionOnlyTranslation()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(-0.2, 0.25, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, new WiggleParameters());

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionTranslationLimits()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(0.0, 0.0);
      plane.addVertex(0.5, 0.0);
      plane.addVertex(0.0, 0.5);
      plane.addVertex(0.5, 0.5);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(-0.2, 0.25, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 0.1;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

      if (visualize)
      {
         addPolygonToArtifacts("Plane", plane, Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      assertTrue(foot == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionTranslationLimitX1()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.0, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 0.0;
      parameters.maxY = 1.0;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionTranslationLimitX2()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(1.0, 1.5, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.0;
      parameters.minY = -1.0;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionTranslationLimitY1()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.0, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.maxX = 1.0;
      parameters.maxY = 0.05;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionTranslationLimitY2()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(1.0, 1.5, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = -1.0;
      parameters.minY = 0.0;
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testKnownResult()
   {
      // this is a regression test - this will check if the expected result is produced.
      ConvexPolygon2d plane = new ConvexPolygon2d();
      plane.addVertex(1.0, 0.0);
      plane.addVertex(0.0, 1.0);
      plane.addVertex(2.0, 0.0);
      plane.addVertex(0.0, 2.0);
      plane.update();

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setTranslationAndIdentityRotation(0.5, 0.05, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.rotationWeight = 0.2;
      parameters.maxX = 0.5;
      parameters.minX = -0.5;
      parameters.maxY = 0.5;
      parameters.minY = -0.5;
      parameters.maxYaw = Math.toRadians(15.0);
      parameters.minYaw = -Math.toRadians(15.0);
      ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, parameters);

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
      expected.add(new Point2D(0.7021375429674444, 0.405444343736243));
      expected.add(new Point2D(0.901582265978714, 0.39055130969485763));
      expected.add(new Point2D(0.8941357489580215, 0.2908289481892227));
      expected.add(new Point2D(0.6946910259467516, 0.3057219822306081));

      for (int i = 0; i < foot.getNumberOfVertices(); i++)
         assertTrue(foot.getVertex(i).epsilonEquals(expected.get(i), 1.0E-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testCompexProjectionArea()
   {
      ConvexPolygon2d plane = new ConvexPolygon2d();
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
         ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
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
         initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

         ConvexPolygon2d foot = PolygonWiggler.wigglePolygon(initialFoot, plane, wiggleParameters);
         assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, plane));
         if (ConvexPolygon2dCalculator.isPolygonInside(initialFoot, 1.0e-5, plane))
            assertTrue(initialFoot.epsilonEquals(foot, 1.0e-10));

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionIntoPlanarRegion1()
   {
      ArrayList<ConvexPolygon2d> planes = new ArrayList<>();
      ConvexPolygon2d plane1 = new ConvexPolygon2d();
      plane1.addVertex(0.0, 0.0);
      plane1.addVertex(0.5, 0.0);
      plane1.addVertex(0.0, 0.5);
      plane1.addVertex(0.5, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2d plane2 = new ConvexPolygon2d();
      plane2.addVertex(-0.6, 0.0);
      plane2.addVertex(-0.1, 0.0);
      plane2.addVertex(-0.6, 0.5);
      plane2.addVertex(-0.1, 0.5);
      plane2.update();
      planes.add(plane2);
      ConvexPolygon2d plane3 = new ConvexPolygon2d();
      plane3.addVertex(-0.25, 0.0);
      plane3.addVertex(0.25, 0.0);
      plane3.addVertex(-0.25, -0.5);
      plane3.addVertex(0.25, -0.5);
      plane3.update();
      planes.add(plane3);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2d foot = new ConvexPolygon2d(initialFoot);
      foot.applyTransformAndProjectToXYPlane(wiggleTransfrom);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionIntoPlanarRegion2()
   {
      ArrayList<ConvexPolygon2d> planes = new ArrayList<>();
      ConvexPolygon2d plane1 = new ConvexPolygon2d();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2d plane2 = new ConvexPolygon2d();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2d foot = new ConvexPolygon2d(initialFoot);
      foot.applyTransformAndProjectToXYPlane(wiggleTransfrom);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionIntoPlanarRegionNoOverlap()
   {
      ArrayList<ConvexPolygon2d> planes = new ArrayList<>();
      ConvexPolygon2d plane1 = new ConvexPolygon2d();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2d plane2 = new ConvexPolygon2d();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(0.1, 0.1, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionIntoPlanarRegionInvalidLimits()
   {
      ArrayList<ConvexPolygon2d> planes = new ArrayList<>();
      ConvexPolygon2d plane1 = new ConvexPolygon2d();
      plane1.addVertex(-0.6, 0.0);
      plane1.addVertex(-0.1, 0.0);
      plane1.addVertex(-0.6, 0.5);
      plane1.addVertex(-0.1, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2d plane2 = new ConvexPolygon2d();
      plane2.addVertex(-0.25, 0.0);
      plane2.addVertex(0.25, 0.0);
      plane2.addVertex(-0.25, -0.5);
      plane2.addVertex(0.25, -0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.09, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = 0.0;
      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoRegion(initialFoot, region, parameters);
      assertTrue(wiggleTransfrom == null);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         showPlotterAndSleep(artifacts);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProjectionIntoPlanarRegionHull()
   {
      ArrayList<ConvexPolygon2d> planes = new ArrayList<>();
      ConvexPolygon2d plane1 = new ConvexPolygon2d();
      plane1.addVertex(0.0, 0.0);
      plane1.addVertex(0.5, 0.0);
      plane1.addVertex(0.0, 0.5);
      plane1.addVertex(0.5, 0.5);
      plane1.update();
      planes.add(plane1);
      ConvexPolygon2d plane2 = new ConvexPolygon2d();
      plane2.addVertex(-0.6, 0.0);
      plane2.addVertex(-0.1, 0.0);
      plane2.addVertex(-0.6, 0.5);
      plane2.addVertex(-0.1, 0.5);
      plane2.update();
      planes.add(plane2);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegion region = new PlanarRegion(transformToWorld, planes);

      ConvexPolygon2d initialFoot = PlanningTestTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.setTranslation(-0.05, 0.05, 0.0);
      initialFoot.applyTransformAndProjectToXYPlane(initialFootTransform);

      RigidBodyTransform wiggleTransfrom = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(initialFoot, region, new WiggleParameters());
      assertFalse(wiggleTransfrom == null);

      ConvexPolygon2d foot = new ConvexPolygon2d(initialFoot);
      foot.applyTransformAndProjectToXYPlane(wiggleTransfrom);

      if (visualize)
      {
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
            addPolygonToArtifacts("Plane" + i, region.getConvexPolygon(i), Color.BLACK);
         addPolygonToArtifacts("InitialFoot", initialFoot, Color.RED);
         addPolygonToArtifacts("Foot", foot, Color.BLUE);
         showPlotterAndSleep(artifacts);
      }

      ConvexPolygon2d hullOfRegion = new ConvexPolygon2d(plane1, plane2);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(foot, 1.0e-5, hullOfRegion));
   }

   private void addPolygonToArtifacts(String name, ConvexPolygon2d polygon, Color color)
   {
      YoFrameConvexPolygon2d yoPlanePolygon = new YoFrameConvexPolygon2d(name + "Polygon", worldFrame, 10, registry);
      artifacts.add(new YoArtifactPolygon(name, yoPlanePolygon , color, false));
      yoPlanePolygon.setConvexPolygon2d(polygon);
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
