package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.ConcavePolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonWiggling.PolygonWigglingTest.showPlotterAndSleep;

public class ConcavePolygonWigglerTest
{
   private static final boolean visualize = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());

   @Test
   public void testSimpleProjection()
   {
      ConcavePolygonWiggler concavePolygonWiggler = new ConcavePolygonWiggler();

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
      wiggleParameters.deltaInside = -0.01;

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
         addLineSegments("Plane", polygon, Color.BLACK, artifacts, registry);
         addLineSegments("InitialFoot", initialFoot, Color.RED, artifacts, registry);
         addLineSegments("Foot", transformedFoothold, Color.BLUE, artifacts, registry);
         addLineSegments("SolverFoot", Vertex2DSupplier.asVertex2DSupplier(concavePolygonWiggler.getTransformedVertices()), Color.GREEN, artifacts, registry);
         showPlotterAndSleep(artifacts);
      }
   }

   @Test
   public void testSawToothProjection()
   {
      ConcavePolygonWiggler concavePolygonWiggler = new ConcavePolygonWiggler();

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
         addLineSegments("Plane", polygon, Color.BLACK, artifacts, registry);
         addLineSegments("InitialFoot", initialFoot, Color.RED, artifacts, registry);
         addLineSegments("Foot", transformedFoothold, Color.BLUE, artifacts, registry);
         addLineSegments("SolverFoot", Vertex2DSupplier.asVertex2DSupplier(concavePolygonWiggler.getTransformedVertices()), Color.GREEN, artifacts, registry);
         showPlotterAndSleep(artifacts);
      }
   }

   @Test
   public void testCrazyPolygon()
   {
      ConcavePolygonWiggler concavePolygonWiggler = new ConcavePolygonWiggler();

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
      initialFootTransform.getTranslation().set(0.0, 0.4, 0.0);
      initialFootTransform.getRotation().setToYawOrientation(Math.toRadians(-40.0));
      initialFoot.applyTransform(initialFootTransform, false);

      WiggleParameters wiggleParameters = new WiggleParameters();
      RigidBodyTransform transform = concavePolygonWiggler.wigglePolygon(initialFoot, polygon, wiggleParameters);

      ConvexPolygon2D transformedFoothold = new ConvexPolygon2D(initialFoot);
      transformedFoothold.applyTransform(transform, false);

      if (visualize)
      {
         addLineSegments("Plane", polygon, Color.BLACK, artifacts, registry);
         addLineSegments("InitialFoot", initialFoot, Color.RED, artifacts, registry);
         addLineSegments("Foot", transformedFoothold, Color.BLUE, artifacts, registry);
         addLineSegments("SolverFoot", Vertex2DSupplier.asVertex2DSupplier(concavePolygonWiggler.getTransformedVertices()), Color.GREEN, artifacts, registry);
         showPlotterAndSleep(artifacts);
      }
   }

   static void addLineSegments(String name, Vertex2DSupplier polygon, Color color, ArtifactList artifacts, YoVariableRegistry registry)
   {
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
         Point2DReadOnly nextVertex = polygon.getVertex((i + 1) % polygon.getNumberOfVertices());
         YoFrameLineSegment2D lineSegment2D = new YoFrameLineSegment2D(name + "LineSegment" + i, worldFrame, registry);
         lineSegment2D.getFirstEndpoint().set(vertex);
         lineSegment2D.getSecondEndpoint().set(nextVertex);

         artifacts.add(new YoArtifactLineSegment2d(name + i, lineSegment2D, color));
      }
   }
}
