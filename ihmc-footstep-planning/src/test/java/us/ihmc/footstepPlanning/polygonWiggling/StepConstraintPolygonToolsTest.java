package us.ihmc.footstepPlanning.polygonWiggling;

import static us.ihmc.footstepPlanning.polygonWiggling.PolygonWigglingTest.showPlotterAndSleep;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StepConstraintPolygonToolsTest
{
   private final Random random = new Random(456789L);
   private final double epsilon = 1e-8;

   private static final boolean visualize = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());

   @Test
   @Disabled
   public void queryAndShowForRandomPolygon()
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

      int numPoints = 10000;
      Random random = new Random();
      for (int i = 0; i < numPoints; i++)
      {
         Point2D queryPoint = new Point2D();
         queryPoint.setX(EuclidCoreRandomTools.nextDouble(random, -0.2, 0.8));
         queryPoint.setY(EuclidCoreRandomTools.nextDouble(random, -0.2, 0.8));
         boolean pointInsidePolygon = StepConstraintPolygonTools.isPointInsidePolygon(polygon, queryPoint);
         Color color = pointInsidePolygon ? Color.GREEN.darker() : Color.BLUE;
         addVertex("v" + i, queryPoint, color, artifacts, registry);
      }

      if (visualize)
      {
         addLineSegments("Plane", polygon, Color.BLACK, artifacts, registry);
         showPlotterAndSleep(artifacts);
      }
   }


   @Test
   public void testIntersectionChecks1()
   {
      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.update();

      ConvexPolygon2D testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(-0.5, -0.5);
      Assertions.assertTrue(StepConstraintPolygonTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(0.0, 1.0);
      Assertions.assertTrue(StepConstraintPolygonTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(1.0, 0.0);
      Assertions.assertTrue(StepConstraintPolygonTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(0.0, -1.0);
      Assertions.assertTrue(StepConstraintPolygonTools.arePolygonsIntersecting(unitSquare, testPolygon));
   }

   @Test
   public void testIntersectionChecks2()
   {
      int numTests = 10000;
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      for (int i = 0; i < numTests; i++)
      {
         ConvexPolygon2D polygon1 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 2.0, 4);
         ConvexPolygon2D polygon2 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 2.0, 4);

         double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(polygon1, polygon2);
         boolean intersectionDetected = StepConstraintPolygonTools.arePolygonsIntersecting(polygon1, polygon2);

         Assertions.assertEquals(intersectionDetected, intersectionArea > 1e-8);
      }
   }

   @Test
   public void testDistanceChecks1()
   {
      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.update();

      double distanceToCheck = 0.15;

      ConvexPolygon2D testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(-1.0 - distanceToCheck, 0.0);
      Assertions.assertTrue(Math.abs(StepConstraintPolygonTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(1.0 + distanceToCheck, 0.0);
      Assertions.assertTrue(Math.abs(StepConstraintPolygonTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(0.0, 1.0 + distanceToCheck);
      Assertions.assertTrue(Math.abs(StepConstraintPolygonTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(0.0, -1.0 - distanceToCheck);
      Assertions.assertTrue(Math.abs(StepConstraintPolygonTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);
   }


   static void addLineSegments(String name, Vertex2DSupplier polygon, Color color, ArtifactList artifacts, YoRegistry registry)
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

   static void addVertex(String name, Point2D vertex, Color color, ArtifactList artifacts, YoRegistry registry)
   {
      YoFramePoint2D framePoint2D = new YoFramePoint2D(name + "Point", worldFrame, registry);
      framePoint2D.set(vertex);
      artifacts.add(new YoArtifactPosition(name, framePoint2D, GraphicType.BALL, color, 0.002));
   }
}