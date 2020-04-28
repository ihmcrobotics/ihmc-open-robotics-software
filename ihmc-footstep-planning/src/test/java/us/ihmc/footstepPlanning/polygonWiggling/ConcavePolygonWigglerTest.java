package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.ConcavePolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PointInPolygonSolver;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

public class ConcavePolygonWigglerTest
{
   private static boolean visualize = false;

   private ConcavePolygonWiggler concavePolygonWiggler;
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry graphicsListRegistry;
   private YoVariableRegistry registry;
   private WiggleParameters wiggleParameters;

   @BeforeEach
   private void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      wiggleParameters = new WiggleParameters();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("testRobot"));
         registry = new YoVariableRegistry(getClass().getSimpleName());
         graphicsListRegistry = new YoGraphicsListRegistry();
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
   public void testSimpleSquare()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.0, 0.5));
      vertexList.add(new Point2D(0.5, 0.5));
      vertexList.add(new Point2D(0.5, 0.0));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(100.0));
      initialFootTransform.getTranslation().set(0.055, 0.25, 0.0);

      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);
   }

   @Test
   public void testSlightlyConcavePolygon()
   {
      List<Point2D> vertexList = new ArrayList<>();
      vertexList.add(new Point2D(0.0, 0.0));
      vertexList.add(new Point2D(0.05, 0.25));
      vertexList.add(new Point2D(0.0, 0.5));
      vertexList.add(new Point2D(0.5, 0.5));
      vertexList.add(new Point2D(0.5, 0.0));
      vertexList.add(new Point2D(0.25, -0.05));
      Vertex2DSupplier polygon = Vertex2DSupplier.asVertex2DSupplier(vertexList);

      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(-30.0));
      initialFootTransform.getTranslation().set(-0.1, -0.3, 0.0);

      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);
   }

   @Test
   public void testSawToothPolygon()
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

      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(10.0));
      initialFootTransform.getTranslation().set(0.6, 0.3, 0.0);

      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);
   }

   @Test
   public void testCrazyPolygon()
   {
      Vertex2DSupplier polygon = getCrazyPolygon1();

      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.getTranslation().set(0.0, 0.4, 0.0);
      initialFootTransform.getRotation().setToYawOrientation(Math.toRadians(-110.0));

      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);
   }

   @Test
   public void testCrazyPolygon2()
   {
      Vertex2DSupplier polygon = getCrazyPolygon2();

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.rotationWeight = 0.02;

      // foot position 1
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(100.0));
      initialFootTransform.getTranslation().set(-1.0, 0.5, 0.0);
      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);

      // foot position 2
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(80.0));
      initialFootTransform.getTranslation().set(-0.3, 0.8, 0.0);
      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);

      // foot position 3
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(0.0));
      initialFootTransform.getTranslation().set(0.8, 0.15, 0.0);
      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);

      // foot position 4
      concavePolygonWiggler.setAlpha(0.25);
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(0.0));
      initialFootTransform.getTranslation().set(0.3, -0.8, 0.0);
      runTests(polygon, initialFootTransform, -0.01, 0.0, 0.01);
   }

   private static Vertex2DSupplier getCrazyPolygon1()
   {
      List<Point2D> concaveHull = new ArrayList<>();
      concaveHull.add(new Point2D(0.0, 0.0));
      concaveHull.add(new Point2D(0.0, 0.6));
      concaveHull.add(new Point2D(0.05, 0.65));
      concaveHull.add(new Point2D(0.25, 0.2));
      concaveHull.add(new Point2D(0.1, 0.65));
      concaveHull.add(new Point2D(0.7, 0.75));
      concaveHull.add(new Point2D(0.2, 0.6));
      concaveHull.add(new Point2D(0.4, 0.6));
      concaveHull.add(new Point2D(0.35, 0.5));
      concaveHull.add(new Point2D(0.4, 0.4));
      concaveHull.add(new Point2D(0.35, 0.3));
      concaveHull.add(new Point2D(0.4, 0.2));
      concaveHull.add(new Point2D(0.35, 0.1));
      concaveHull.add(new Point2D(0.4, 0.0));

      return Vertex2DSupplier.asVertex2DSupplier(concaveHull);
   }

   private static Vertex2DSupplier getCrazyPolygon2()
   {
      List<Point2D> concaveHull = new ArrayList<>();
      concaveHull.add(new Point2D(1.050, 0.030));
      concaveHull.add(new Point2D(0.790, 0.160));
      concaveHull.add(new Point2D(0.990, 0.460));
      concaveHull.add(new Point2D(0.800, 0.660));
      concaveHull.add(new Point2D(0.620, 0.730));
      concaveHull.add(new Point2D(0.410, 0.820));
      concaveHull.add(new Point2D(0.390, 1.110));
      concaveHull.add(new Point2D(0.090, 0.910));
      concaveHull.add(new Point2D(-0.130, 1.320));
      concaveHull.add(new Point2D(-0.250, 0.900));
      concaveHull.add(new Point2D(-0.310, 0.650));
      concaveHull.add(new Point2D(-0.740, 1.100));
      concaveHull.add(new Point2D(-0.550, 0.500));
      concaveHull.add(new Point2D(-0.870, 0.470));
      concaveHull.add(new Point2D(-0.910, 0.260));
      concaveHull.add(new Point2D(-0.640, 0.080));
      concaveHull.add(new Point2D(-1.110, 0.000));
      concaveHull.add(new Point2D(-1.390, -0.180));
      concaveHull.add(new Point2D(-1.330, -0.510));
      concaveHull.add(new Point2D(-0.550, -0.350));
      concaveHull.add(new Point2D(-0.660, -0.640));
      concaveHull.add(new Point2D(-0.590, -0.840));
      concaveHull.add(new Point2D(-0.270, -0.570));
      concaveHull.add(new Point2D(-0.260, -1.140));
      concaveHull.add(new Point2D(-0.050, -0.730));
      concaveHull.add(new Point2D(0.180, -1.180));
      concaveHull.add(new Point2D(0.230, -0.720));
      concaveHull.add(new Point2D(0.410, -0.660));
      concaveHull.add(new Point2D(0.640, -0.630));
      concaveHull.add(new Point2D(1.040, -0.650));
      concaveHull.add(new Point2D(1.000, -0.470));
      concaveHull.add(new Point2D(0.830, -0.180));

      return Vertex2DSupplier.asVertex2DSupplier(concaveHull);
   }

   private void runTests(Vertex2DSupplier polygon, RigidBodyTransform initialFootTransform, double... insideDeltas)
   {
      for (int i = 0; i < insideDeltas.length; i++)
      {
         wiggleParameters.deltaInside = insideDeltas[i];
         runTest(polygon, initialFootTransform);
      }
   }

   private void runTest(Vertex2DSupplier polygon, RigidBodyTransform initialFootTransform)
   {
      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      initialFoot.applyTransform(initialFootTransform, false);
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);

      ConvexPolygon2D transformedFoot = new ConvexPolygon2D(initialFoot);
      transformedFoot.applyTransform(transform, false);

      if (!visualize)
      {
         for (int i = 0; i < transformedFoot.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = transformedFoot.getVertex(i);
            boolean isInPolygon = PointInPolygonSolver.isPointInsidePolygon(polygon, vertex);
            double distanceFromPolygon = distanceFromPolygon(polygon, vertex);
            double epsilon = concavePolygonWiggler.getGradientMagnitudeToTerminate() / concavePolygonWiggler.getAlpha();

            double signedDistanceFromPolygon = isInPolygon ? - distanceFromPolygon : distanceFromPolygon;
            Assertions.assertTrue(signedDistanceFromPolygon < - wiggleParameters.deltaInside + epsilon);
         }
      }
   }

   private static double distanceFromPolygon(Vertex2DSupplier polygon, Point2DReadOnly vertex)
   {
      double minDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         double distance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(vertex, polygon.getVertex(i), polygon.getVertex((i + 1) % polygon.getNumberOfVertices()));
         if (distance < minDistance)
         {
            minDistance = distance;
         }
      }

      return minDistance;
   }

   public static void main(String[] args)
   {
      runTimingTest();
   }

   private static void runTimingTest()
   {
      List<TimingTestConfiguration> testConfigs = new ArrayList<>();
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon1(), 0.0, 0.4, Math.toRadians(-110.0), -0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon1(), 0.0, 0.4, Math.toRadians(-110.0), 0.0));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon1(), 0.0, 0.4, Math.toRadians(-110.0), 0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -1.0, 0.5, Math.toRadians(100.0), -0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -1.0, 0.5, Math.toRadians(100.0), 0.0));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -1.0, 0.5, Math.toRadians(100.0), 0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -0.3, 1.4, Math.toRadians(80.0), -0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -0.3, 1.4, Math.toRadians(80.0), 0.0));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), -0.3, 1.4, Math.toRadians(80.0), 0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 1.1, 0.15, Math.toRadians(0.0), -0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 1.1, 0.15, Math.toRadians(0.0), 0.0));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 1.1, 0.15, Math.toRadians(0.0), 0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 0.9, -0.9, Math.toRadians(0.0), -0.01));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 0.9, -0.9, Math.toRadians(0.0), 0.0));
      testConfigs.add(new TimingTestConfiguration(getCrazyPolygon2(), 0.9, -0.9, Math.toRadians(0.0), 0.01));

      ConcavePolygonWiggler wiggler = new ConcavePolygonWiggler();
      int numWarmupLoops = 5;
      int numTestLoops = 1;

      long[] concaveTimes = new long[testConfigs.size()];
      long[] convexTimes = new long[testConfigs.size()];

      // warmup
      for (int i = 0; i < numWarmupLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);
            wiggler.wigglePolygon(testConfig.initialPolygon, testConfig.concavePolygon, testConfig.wiggleParameters);
         }
      }

      long start = System.nanoTime();
      for (int i = 0; i < numTestLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);
            long s = System.nanoTime();
            wiggler.wigglePolygon(testConfig.initialPolygon, testConfig.concavePolygon, testConfig.wiggleParameters);
            concaveTimes[j]  = System.nanoTime() - s;
         }
      }
      long diff = System.nanoTime() - start;

      long perTest = diff / testConfigs.size();
      long perTestUs = perTest / (numTestLoops * 1000);
      System.out.println("Concave polygon wiggler microseconds per run: " + perTestUs);

      // warmup
      for (int i = 0; i < numWarmupLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);
            PolygonWiggler.findWiggleTransform(testConfig.initialPolygon, testConfig.convexPolygon, testConfig.wiggleParameters);
         }
      }

      start = System.nanoTime();
      for (int i = 0; i < numTestLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);
            long s = System.nanoTime();
            PolygonWiggler.findWiggleTransform(testConfig.initialPolygon, testConfig.convexPolygon, testConfig.wiggleParameters);
            convexTimes[j]  = System.nanoTime() - s;
         }
      }
      diff = System.nanoTime() - start;

      for (int i = 0; i < testConfigs.size(); i++)
      {
         System.out.println(( i + 1 ) + "\t " + concaveTimes[i] + " \t " + convexTimes[i]);
      }

      perTest = diff / testConfigs.size();
      perTestUs = perTest /  (numTestLoops * 1000);
      System.out.println("Convex polygon wiggler microseconds per run: " + perTestUs);
   }

   private static class TimingTestConfiguration
   {
      Vertex2DSupplier concavePolygon;
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      ConvexPolygon2D initialPolygon;
      WiggleParameters wiggleParameters = new WiggleParameters();

      public TimingTestConfiguration(Vertex2DSupplier concavePolygon, double dx, double dy, double dYaw, double deltaInside)
      {
         this.concavePolygon = concavePolygon;
         initialPolygon = PlannerTools.createDefaultFootPolygon();

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(dYaw);
         transform.getTranslation().set(dx, dy, 0.0);
         initialPolygon.applyTransform(transform, false);
         wiggleParameters.deltaInside = deltaInside;
         wiggleParameters.rotationWeight = 0.05;

         for (int i = 0; i < concavePolygon.getNumberOfVertices(); i++)
         {
            convexPolygon.addVertex(concavePolygon.getVertex(i));
         }
         convexPolygon.update();
      }
   }
}
