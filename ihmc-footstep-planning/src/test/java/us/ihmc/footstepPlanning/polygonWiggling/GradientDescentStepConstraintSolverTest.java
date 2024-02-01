package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.*;
import us.ihmc.commonWalkingControlModules.polygonWiggling.*;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class GradientDescentStepConstraintSolverTest
{
   private static boolean visualize = false;

   private GradientDescentStepConstraintSolver gradientDescentStepConstraintSolver;
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry graphicsListRegistry;
   private YoRegistry registry;
   private WiggleParameters wiggleParameters;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      wiggleParameters = new WiggleParameters();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("testRobot"));
         scs.setGroundVisible(false);
         registry = new YoRegistry(getClass().getSimpleName());
         graphicsListRegistry = new YoGraphicsListRegistry();
         gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver(scs, graphicsListRegistry, registry);
         graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.getRootRegistry().addChild(registry);
         scs.startOnAThread();
      }
      else
      {
         gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver();
      }
   }

   @AfterEach
   public void after()
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
      gradientDescentStepConstraintSolver.setAlpha(0.25);
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
      GradientDescentStepConstraintInput input = new GradientDescentStepConstraintInput();

      input.setInitialStepPolygon(initialFoot);
      input.setPolygonToWiggleInto(polygon);
      input.setWiggleParameters(wiggleParameters);

      RigidBodyTransform transform = gradientDescentStepConstraintSolver.wigglePolygon(input);

      ConvexPolygon2D transformedFoot = new ConvexPolygon2D(initialFoot);
      transformedFoot.applyTransform(transform, false);

      if (!visualize)
      {
         for (int i = 0; i < transformedFoot.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = transformedFoot.getVertex(i);
            boolean isInPolygon = StepConstraintPolygonTools.isPointInsidePolygon(polygon, vertex);
            double distanceFromPolygon = distanceFromPolygon(polygon, vertex);
            double epsilon = gradientDescentStepConstraintSolver.getGradientMagnitudeToTerminate() / gradientDescentStepConstraintSolver.getAlpha();

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

      GradientDescentStepConstraintSolver wiggler = new GradientDescentStepConstraintSolver();
      int numWarmupLoops = 5;
      int numTestLoops = 1;

      long[] concaveTimes = new long[testConfigs.size()];
      long[] convexTimes = new long[testConfigs.size()];

      GradientDescentStepConstraintInput input = new GradientDescentStepConstraintInput();

      // warmup
      for (int i = 0; i < numWarmupLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);

            input.setInitialStepPolygon(testConfig.initialPolygon);
            input.setPolygonToWiggleInto(testConfig.concavePolygon);
            input.setWiggleParameters(testConfig.wiggleParameters);
            wiggler.wigglePolygon(input);
         }
      }

      long start = System.nanoTime();
      for (int i = 0; i < numTestLoops; i++)
      {
         for (int j = 0; j < testConfigs.size(); j++)
         {
            TimingTestConfiguration testConfig = testConfigs.get(j);
            long s = System.nanoTime();

            input.setInitialStepPolygon(testConfig.initialPolygon);
            input.setPolygonToWiggleInto(testConfig.concavePolygon);
            input.setWiggleParameters(testConfig.wiggleParameters);
            wiggler.wigglePolygon(input);

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

   @Disabled
   @Test
   public void testShinCollisionOnSimpleStairs()
   {
      double stepHeight = 0.3;
      double stepLength = 0.25;
      double stepWidth = 0.8;
      int steps = 4;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(4.0, 4.0);
      planarRegionsListGenerator.translate(0.5 * stepLength, 0.0, 0.0);

      for (int i = 0; i < steps; i++)
      {
         planarRegionsListGenerator.translate(stepLength, 0.0, stepHeight);
         planarRegionsListGenerator.rotate(Math.PI, Axis3D.Z);
         planarRegionsListGenerator.addRectangle(stepLength, stepWidth);
         planarRegionsListGenerator.rotate(-Math.PI, Axis3D.Z);
      }

      PlanarRegionsList stairCaseRegions = planarRegionsListGenerator.getPlanarRegionsList();
      PlanarRegion firstStep = stairCaseRegions.getPlanarRegion(1);

      ConvexPolygon2D footPolygon = PlannerTools.createDefaultFootPolygon();
      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = -0.1;

      double shinPitch = Math.toRadians(25.0);
      double shinRadius = 0.05;
      double shinLength = 0.4;
      Cylinder3D legCylinder = new Cylinder3D(shinLength, shinRadius);

      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.translate(0.0, 0.0, 0.05);
      transformGenerator.rotate(shinPitch, Axis3D.Y);
      transformGenerator.translate(0.0, 0.0, 0.5 * shinLength);
      RigidBodyTransform transformToSoleFrame = transformGenerator.getRigidBodyTransformCopy();
      gradientDescentStepConstraintSolver.setLegCollisionShape(legCylinder, transformToSoleFrame);

      RigidBodyTransform footTransformInLocal = new RigidBodyTransform();
      footTransformInLocal.appendYawRotation(Math.toRadians(180.0));
//      footTransformInLocal.appendTranslation(0.02, 0.01, 0.0);

      footPolygon.applyTransform(footTransformInLocal);

      GradientDescentStepConstraintInput input = new GradientDescentStepConstraintInput();
      input.setInitialStepPolygon(footPolygon);
      input.setPlanarRegion(firstStep);
      input.setWiggleParameters(wiggleParameters);
      input.setFootstepInRegionFrame(footTransformInLocal);
      input.setPlanarRegionsList(stairCaseRegions);

      gradientDescentStepConstraintSolver.wigglePolygon(input);
   }

   @Disabled
   @Test
   public void testShinCollisionOnDataSet()
   {
      DataSet stairsDataSet = DataSetIOTools.loadDataSet(DataSetName._20200513_151318_StairsIHMC_Bottom);
      PlanarRegionsList stairsRegions = stairsDataSet.getPlanarRegionsList();

      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      RigidBodyTransform footTransformInLocal = new RigidBodyTransform();

      int stepIndex = 8;
      PlanarRegion step = packStep(stepIndex, stairsRegions, footPolygon, footTransformInLocal);

      double shinPitch = Math.toRadians(25.0);
      double shinRadius = 0.05;
      double shinLength = 0.4;
      Cylinder3D legCylinder = new Cylinder3D(shinLength, shinRadius);

      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.translate(0.0, 0.0, 0.05);
      transformGenerator.rotate(shinPitch, Axis3D.Y);
      transformGenerator.translate(0.0, 0.0, 0.5 * shinLength);
      RigidBodyTransform transformToSoleFrame = transformGenerator.getRigidBodyTransformCopy();

      gradientDescentStepConstraintSolver.setLegCollisionShape(legCylinder, transformToSoleFrame);

      GradientDescentStepConstraintInput input = new GradientDescentStepConstraintInput();
      input.setInitialStepPolygon(footPolygon);
      input.setPlanarRegion(step);
      input.setWiggleParameters(new WiggleParameters());
      input.setFootstepInRegionFrame(footTransformInLocal);
      input.setPlanarRegionsList(stairsRegions);

      gradientDescentStepConstraintSolver.wigglePolygon(input);
   }

   private PlanarRegion packStep(int stepIndex, PlanarRegionsList stairsRegions, ConvexPolygon2D footPolygon, RigidBodyTransform footTransformInLocal)
   {
      if (stepIndex == 0)
      {
         footPolygon.addVertex(-0.658,  0.307 );
         footPolygon.addVertex(-0.456,  0.394 );
         footPolygon.addVertex(-0.418,  0.291 );
         footPolygon.addVertex(-0.629,  0.228 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.342, -0.940));
         footTransformInLocal.getTranslation().set(-0.54, 0.305, 0.0);

         return stairsRegions.getRegionWithId(1477610134);
      }
      if (stepIndex == 1)
      {
         footPolygon.addVertex(-0.094, -0.230 );
         footPolygon.addVertex( 0.091, -0.109 );
         footPolygon.addVertex( 0.146, -0.204 );
         footPolygon.addVertex(-0.051, -0.304 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.5, -0.866));
         footTransformInLocal.getTranslation().set(0.023, -0.212, 0.0);

         return stairsRegions.getRegionWithId(1207074051);
      }
      if (stepIndex == 2)
      {
         footPolygon.addVertex(-0.756,  0.432 );
         footPolygon.addVertex(-0.741,  0.516 );
         footPolygon.addVertex(-0.523,  0.490 );
         footPolygon.addVertex(-0.542,  0.382 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(0.174, -0.985));
         footTransformInLocal.getTranslation().set(-0.640, 0.455, 0.0);

         return stairsRegions.getRegionWithId(1477610134);
      }
      if (stepIndex == 3)
      {
         footPolygon.addVertex(-0.708,  0.157 );
         footPolygon.addVertex(-0.506,  0.244 );
         footPolygon.addVertex(-0.468,  0.141 );
         footPolygon.addVertex(-0.679,  0.078 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.342, -0.940));
         footTransformInLocal.getTranslation().set(-0.590, 0.155, 0.0);

         return stairsRegions.getRegionWithId(1477610134);
      }
      if (stepIndex == 6)
      {
         footPolygon.addVertex(-0.146,  0.255 );
         footPolygon.addVertex(-0.131,  0.339 );
         footPolygon.addVertex( 0.088,  0.313 );
         footPolygon.addVertex( 0.069,  0.205 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(0.174, -0.985));
         footTransformInLocal.getTranslation().set(-0.030, 0.278, 0.0);

         return stairsRegions.getRegionWithId(1695211678);
      }
      if (stepIndex == 7)
      {
         footPolygon.addVertex(-0.126,  0.176 );
         footPolygon.addVertex( 0.088,  0.227 );
         footPolygon.addVertex( 0.107,  0.118 );
         footPolygon.addVertex(-0.112,  0.092 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.174, -0.985));
         footTransformInLocal.getTranslation().set(-0.030, 0.278, 0.0);

         return stairsRegions.getRegionWithId(1695211678);
      }
      if (stepIndex == 8)
      {
         footPolygon.addVertex(-0.156,  0.416 );
         footPolygon.addVertex( 0.046,  0.503 );
         footPolygon.addVertex( 0.084,  0.400 );
         footPolygon.addVertex(-0.127,  0.337 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.342, -0.940));
         footTransformInLocal.getTranslation().set(-0.030, 0.278, 0.0);

         return stairsRegions.getRegionWithId(507821362);
      }
      if (stepIndex == 9)
      {
         footPolygon.addVertex( 0.408,  0.115 );
         footPolygon.addVertex( 0.623,  0.165 );
         footPolygon.addVertex( 0.642,  0.057 );
         footPolygon.addVertex( 0.423,  0.031 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(-0.174, -0.985));
         footTransformInLocal.getTranslation().set(0.524, 0.092, 0.0);

         return stairsRegions.getRegionWithId(1816290613);
      }
      if (stepIndex == 10)
      {
         footPolygon.addVertex( 0.264,  0.335 );
         footPolygon.addVertex( 0.484,  0.347 );
         footPolygon.addVertex( 0.484,  0.237 );
         footPolygon.addVertex( 0.264,  0.250 );
         footPolygon.update();

         footTransformInLocal.getRotation().setToYawOrientation(EuclidCoreTools.atan2(0.0, -1.0));
         footTransformInLocal.getTranslation().set(0.374, 0.292, 0.0);

         return stairsRegions.getRegionWithId(1816290613);
      }

      throw new RuntimeException("dataa not present for step index" + stepIndex);
   }
}
