package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.ConcavePolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

public class ConcavePolygonWigglerTest
{
   private static final boolean visualize = true;

   private ConcavePolygonWiggler concavePolygonWiggler;
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("testRobot"));

   @BeforeEach
   private void setup()
   {
      if (visualize)
      {
         concavePolygonWiggler = new ConcavePolygonWiggler(scs, graphicsListRegistry, registry);
         graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.getRootRegistry().addChild(registry);
         scs.startOnAThread();
      }
      else
      {
         concavePolygonWiggler = new ConcavePolygonWiggler();
      }
   }

   @AfterEach
   private void after()
   {
      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testSimpleProjection()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.05, 0.25));
      vertexList.add(new Point2D(0.0, 0.5));
      vertexList.add(new Point2D(0.5, 0.5));
      vertexList.add(new Point2D(0.5, 0.0));
      vertexList.add(new Point2D(0.25, -0.05));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.getTranslation().set(-0.1, -0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = 0.01;

      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);
   }

   @Test
   public void testSawToothProjection()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.0, 0.6));
      vertexList.add(new Point2D(0.4, 0.6));
      vertexList.add(new Point2D(0.35, 0.5));
      vertexList.add(new Point2D(0.4, 0.4));
      vertexList.add(new Point2D(0.35, 0.3));
      vertexList.add(new Point2D(0.4, 0.2));
      vertexList.add(new Point2D(0.35, 0.1));
      vertexList.add(new Point2D(0.4, 0.0));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(10.0));
      initialFootTransform.getTranslation().set(0.6, 0.3, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = 0.01;
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);
   }

   @Test
   public void testCrazyPolygon()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.0, 0.6));
      vertexList.add(new Point2D(0.05, 0.65));
      vertexList.add(new Point2D(0.25, 0.2));
      vertexList.add(new Point2D(0.1, 0.65));
      vertexList.add(new Point2D(0.7, 0.75));
      vertexList.add(new Point2D(0.2, 0.6));
      vertexList.add(new Point2D(0.4, 0.6));
      vertexList.add(new Point2D(0.35, 0.5));
      vertexList.add(new Point2D(0.4, 0.4));
      vertexList.add(new Point2D(0.35, 0.3));
      vertexList.add(new Point2D(0.4, 0.2));
      vertexList.add(new Point2D(0.35, 0.1));
      vertexList.add(new Point2D(0.4, 0.0));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.getTranslation().set(0.0, 0.5, 0.0);
      initialFootTransform.getRotation().setToYawOrientation(Math.toRadians(-90.0));
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);
   }

   @Test
   public void testSimpleSquare()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.0, 0.5));
      vertexList.add(new Point2D(0.5, 0.5));
      vertexList.add(new Point2D(0.5, 0.0));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(100.0));
      initialFootTransform.getTranslation().set(0.055, 0.25, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);
   }

   @Test
   public void testCrazyPolygon2()
   {
      List<Point2D> concaveHull = new ArrayList<>();
      concaveHull.add(new Point2D( 1.050,  0.030 ));
      concaveHull.add(new Point2D( 0.790,  0.160 ));
      concaveHull.add(new Point2D( 0.990,  0.460 ));
      concaveHull.add(new Point2D( 0.800,  0.660 ));
      concaveHull.add(new Point2D( 0.620,  0.730 ));
      concaveHull.add(new Point2D( 0.410,  0.820 ));
      concaveHull.add(new Point2D( 0.390,  1.110 ));
      concaveHull.add(new Point2D( 0.090,  0.910 ));
      concaveHull.add(new Point2D(-0.130,  1.320 ));
      concaveHull.add(new Point2D(-0.250,  0.900 ));
      concaveHull.add(new Point2D(-0.310,  0.650 ));
      concaveHull.add(new Point2D(-0.740,  1.100 ));
      concaveHull.add(new Point2D(-0.550,  0.500 ));
      concaveHull.add(new Point2D(-0.870,  0.470 ));
      concaveHull.add(new Point2D(-0.910,  0.260 ));
      concaveHull.add(new Point2D(-0.640,  0.080 ));
      concaveHull.add(new Point2D(-1.110,  0.000 ));
      concaveHull.add(new Point2D(-1.390, -0.180 ));
      concaveHull.add(new Point2D(-1.330, -0.510 ));
      concaveHull.add(new Point2D(-0.550, -0.350 ));
      concaveHull.add(new Point2D(-0.660, -0.640 ));
      concaveHull.add(new Point2D(-0.590, -0.840 ));
      concaveHull.add(new Point2D(-0.270, -0.570 ));
      concaveHull.add(new Point2D(-0.260, -1.140 ));
      concaveHull.add(new Point2D(-0.050, -0.730 ));
      concaveHull.add(new Point2D( 0.180, -1.180 ));
      concaveHull.add(new Point2D( 0.230, -0.720 ));
      concaveHull.add(new Point2D( 0.410, -0.660 ));
      concaveHull.add(new Point2D( 0.640, -0.630 ));
      concaveHull.add(new Point2D( 1.040, -0.650 ));
      concaveHull.add(new Point2D( 1.000, -0.470 ));
      concaveHull.add(new Point2D( 0.830, -0.180 ));

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.rotationWeight = 0.02;

      // foot position 1
      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(100.0));
      initialFootTransform.getTranslation().set(-1.0, 0.5, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, Vertex2DSupplier.asVertex2DSupplier(concaveHull), wiggleParameters);

      // foot position 2
      initialFoot = PlannerTools.createDefaultFootPolygon();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(80.0));
      initialFootTransform.getTranslation().set(-0.3, 0.8, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);
      transform = concavePolygonWiggler.wigglePolygon(initialFoot, Vertex2DSupplier.asVertex2DSupplier(concaveHull), wiggleParameters);

      // foot position 3
      initialFoot = PlannerTools.createDefaultFootPolygon();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(0.0));
      initialFootTransform.getTranslation().set(0.8, 0.15, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);
      transform = concavePolygonWiggler.wigglePolygon(initialFoot, Vertex2DSupplier.asVertex2DSupplier(concaveHull), wiggleParameters);

      // foot position 4
      initialFoot = PlannerTools.createDefaultFootPolygon();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(0.0));
      initialFootTransform.getTranslation().set(-0.65, -0.42, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);
      transform = concavePolygonWiggler.wigglePolygon(initialFoot, Vertex2DSupplier.asVertex2DSupplier(concaveHull), wiggleParameters);

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);

   }
}
