package us.ihmc.footstepPlanning.polygonWiggling;

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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private ConcavePolygonWiggler concavePolygonWiggler;
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());
   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("testRobot"));

   @BeforeEach
   private void setup()
   {
      concavePolygonWiggler = new ConcavePolygonWiggler(scs, graphicsListRegistry, registry);
//      graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());
//      scs.addYoGraphicsListRegistry(graphicsListRegistry);
//      scs.getRootRegistry().addChild(registry);
//      scs.startOnAThread();
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

      int warmsup = 5;
      for (int i = 0; i < warmsup; i++)
      {
         concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);
      }

      int n = 20;
      long start = System.nanoTime();
      for (int i = 0; i < n; i++)
      {
         concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);
      }
      long stop = System.nanoTime();

      long perRun = (stop - start) / n;
      System.out.println(perRun);

//      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
//      transformedFoothold.applyTransform(transform, false);
//
//      long diff = stop - start;
//      System.out.println("time: " + diff + "ns");
//      System.out.println(transform);

//      if (visualize)
//      {
//         ThreadTools.sleepForever();
//      }
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

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
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

      ConvexPolygon2D initialFoot = PlannerTools.createDefaultFootPolygon();
      RigidBodyTransform initialFootTransform = new RigidBodyTransform();
      initialFootTransform.setRotationYawAndZeroTranslation(Math.toRadians(100.0));
      initialFootTransform.getTranslation().set(0.055, 0.25, 0.0);
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.rotationWeight = 0.1;

      long start = System.nanoTime();
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);
      long stop = System.nanoTime();

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);

      long diff = stop - start;
      System.out.println("time: " + diff + "ns");
      System.out.println(transform);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

}
