package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.*;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

public class ConvexPolygon2DTest
{
   private static final boolean visualize = false;
   static final double EPSILON = 1.0e-7;

   @Test
   public void testSimpleGiftWrappingError()
   {
      List<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.0, 0.0));
      points.add(new Point2D(0.1, 0.05));
      points.add(new Point2D(0.2, 0.1));
      points.add(new Point2D(0.1, 0.1));
      points.add(new Point2D(0.2, 0.2));

      GiftWrappingVisualizer visualizer = visualize ? new GiftWrappingVisualizer(points.size()) : null;
      int numberOfPoints = inPlaceGiftWrapConvexHull2D(visualizer, points, points.size());
      assertEquals(points.size(), numberOfPoints);
   }

   @Test
   public void testTerriblePoint()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double snapAreaResolution = 0.2;

      // get a polygon transformed to a random location.
      RigidBodyTransform transform = new RigidBodyTransform();
      int gridXIndex = 19;//100;//-76; //-6;
      int gridYIndex = -32;//-45;//-76;// -59;
      int yawIndex = 4;//2;//22;// 33;
      transform.getTranslation().set(LatticePoint.gridSizeXY * gridXIndex, LatticePoint.gridSizeXY * gridYIndex, 0.0);
      transform.getRotation().appendYawRotation(yawIndex * LatticePoint.gridSizeYaw);

      ConvexPolygon2D transformedPolygon = new ConvexPolygon2D(basePolygon);
      transformedPolygon.applyTransform(transform);

      // Here we want to collect all the points  under the foothold similar to what is done during snapping, but slightly different by assuming they're all
      // valid
      List<Point3D> footPointsInEnvironment = new ArrayList<>();
      Point2DReadOnly corner0 = transformedPolygon.getVertex(0);
      Point2DReadOnly corner1 = transformedPolygon.getVertex(1);
      Point2DReadOnly corner2 = transformedPolygon.getVertex(2);
      Point2DReadOnly corner3 = transformedPolygon.getVertex(3);

      Point2D pointOnEdge1 = new Point2D();
      Point2D pointOnEdge2 = new Point2D();
      Point2D footPointToSnap = new Point2D();

      double height = RandomNumbers.nextDouble(random, 1.0);

      for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
      {
         pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
         pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

         for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
         {
            footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

            Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);
            footPointsInEnvironment.add(point);
         }
      }

      // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
      List<Point2D> sortedPoints = footPointsInEnvironment.stream().map(Point2D::new).collect(Collectors.toList());
      //      int numberOfPoints = inPlaceGiftWrapConvexHull2D(sortedPoints, sortedPoints.size());
      //      ThreadTools.sleepForever();

      List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                            {
                                                                               Point3D transformedPoint = new Point3D(point);
                                                                               transformedPoint.applyInverseTransform(transform);
                                                                               return new Point2D(transformedPoint);
                                                                            }).collect(Collectors.toList());

      GiftWrappingVisualizer visualizer = visualize ? new GiftWrappingVisualizer(footPointsInFoot.size()) : null;
      ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
      inPlaceGiftWrapConvexHull2D(visualizer, footPointsInFoot, footPointsInFoot.size());
      assertPoint2DsInside(visualizer, polygon2D, footPointsInFoot, 1e-7);
   }

   public static int inPlaceGiftWrapConvexHull2D(GiftWrappingVisualizer visualizer, List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (visualizer != null)
         visualizer.setPointCloud(vertices, numberOfVertices);

      if (numberOfVertices == 0)
         return 0;

      checkNumberOfVertices(vertices, numberOfVertices);

      /*
       * Set the first vertex to be the one with the lowest x-coordinate. If the lowest x-coordinate
       * exists in more than one vertex in the list, the vertex with the highest y-coordinate out of the
       * candidates is chosen.
       */
      Collections.swap(vertices, findMinXMaxYVertexIndex(vertices, numberOfVertices), 0);
      if (visualizer != null)
         visualizer.setPointCloud(vertices, numberOfVertices);

      Point2DReadOnly firstVertex = vertices.get(0);

      // This is the last index that belongs to the convex polygon that has been found so far.
      int lastHullVertexIndex = 0;

      while (lastHullVertexIndex < numberOfVertices - 1)
      {
         Point2DReadOnly lastHullVertex = vertices.get(lastHullVertexIndex);
         if (visualizer != null)
            visualizer.setLastHullVertexIndex(lastHullVertexIndex);

         // Index to keep track of the potential next vertex of the polygon.
         int candidateIndex = lastHullVertexIndex + 1;
         Point2DReadOnly candidateVertex = vertices.get(candidateIndex);

         if (visualizer != null)
         {
            visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
            visualizer.updateViz(AlgorithmPhase.CHECKING_FOR_TOO_CLOSE_START);
         }

         while (candidateVertex.epsilonEquals(lastHullVertex, EPSILON))
         { // Remove any duplicate vertices between here and the end of the list. We do this by swapping this vertex to the last of the list, and hten making
            // the working list shorter
            Collections.swap(vertices, candidateIndex, --numberOfVertices);
            if (visualizer != null)
               visualizer.setPointCloud(vertices, numberOfVertices);
            candidateVertex = vertices.get(candidateIndex);

            if (visualizer != null)
            {
               visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
               visualizer.setQueryRejection(QueryRejection.TOO_CLOSE);
               visualizer.updateViz(AlgorithmPhase.REJECTING_CANDIDATE);
            }

            if (numberOfVertices == 1)
            {
               if (visualizer != null)
               {
                  visualizer.setPerimeter(vertices, numberOfVertices);
                  visualizer.updateViz(AlgorithmPhase.REMOVED_ALL);
               }

               return numberOfVertices;
            }
         }
         if (visualizer != null)
         {
            visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
            visualizer.updateViz(AlgorithmPhase.CHECKING_FOR_NONCOLLINEAR_START);
         }

         while ((lastHullVertexIndex >= 1 && isCandidatePointCollinearInOppositeDirectionWithTheFirstPoint(lastHullVertex,
                                                                                                           vertices.get(lastHullVertexIndex - 1),
                                                                                                           candidateVertex,
                                                                                                           EPSILON)))
         { // The next candidate vertex isn't valid because it's collinear, but may be valid for a future point, so we want to keep it in scope.
            candidateIndex++;
            candidateVertex = vertices.get(candidateIndex);

            if (visualizer != null)
            {
               visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
               visualizer.setQueryRejection(QueryRejection.COLLINEAR);
               visualizer.updateViz(AlgorithmPhase.REJECTING_CANDIDATE);
            }
         }

         if (visualizer != null)
         {
            visualizer.setPerimeter(vertices, lastHullVertexIndex + 1);
            visualizer.updateViz(AlgorithmPhase.UPDATING_PERIMETER);
            visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
            visualizer.updateViz(AlgorithmPhase.UPDATING_CANDIDATE);
         }


         // Loop through all the other vertices. Create a line segment starting at the last hull vertex, and ending at the candidate vertex.  If all the other
         // points are to the
         for (int vertexIndex = candidateIndex + 1; vertexIndex <= numberOfVertices; )
         {
            int wrappedIndex = EuclidCoreTools.wrap(vertexIndex, numberOfVertices);
            Point2DReadOnly vertex = vertices.get(wrappedIndex);

            if (visualizer != null)
            {
               visualizer.setQueryPoint(vertex, wrappedIndex);
               visualizer.updateViz(AlgorithmPhase.UPDATING_QUERY);
            }

            if (vertex.epsilonEquals(candidateVertex, EPSILON) || wrappedIndex != 0 && vertex.epsilonEquals(firstVertex, EPSILON))
            { // Remove duplicate vertices
               Collections.swap(vertices, wrappedIndex, --numberOfVertices);
               if (visualizer != null)
                  visualizer.setPointCloud(vertices, numberOfVertices);

               /*
                * Restart iteration without incrementing the vertexIndex, so the wrappedIndex can be updated
                * properly as the numberOfVertices just changed.
                */
               continue;
            }

            if (isPoint2DOnLeftSideOfLine2D(vertex, lastHullVertex, candidateVertex))
            { // vertex is located outside => candidateVertex is not the next polygon vertex, vertex might be though.

               // Check that we're not trying to finish the loop. If we are trying to finish the loop, make sure we're not doubling back, as this is a failure
               // case.
               if (lastHullVertexIndex < 1 || !isCandidatePointCollinearInOppositeDirectionWithTheFirstPoint(lastHullVertex,
                                                                                                             vertices.get(lastHullVertexIndex - 1),
                                                                                                             vertex,
                                                                                                             EPSILON))
               {
                  candidateIndex = wrappedIndex;
                  candidateVertex = vertex;


                  if (visualizer != null)
                  {
                     visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
                     visualizer.updateViz(AlgorithmPhase.UPDATING_CANDIDATE);
                  }
               }
               else if (visualizer != null)
               {
                  visualizer.setQueryRejection(QueryRejection.COLLINEAR);
                  visualizer.updateViz(AlgorithmPhase.REJECTING_QUERY);
               }
            }
            else if (visualizer != null)
            {
               visualizer.setQueryRejection(QueryRejection.TO_THE_RIGHT);
               visualizer.updateViz(AlgorithmPhase.REJECTING_QUERY);
            }

            vertexIndex++;
         }

         if (candidateIndex == 0)
         {
            /*
             * Got back to the first vertex of the polygon: we're done and the size of the polygon is defined by
             * the index of the last vertex found.
             */
            if (visualizer != null)
            {
               visualizer.setPerimeter(vertices, lastHullVertexIndex + 1);
               visualizer.updateViz(AlgorithmPhase.FINISHED_LOOP);
            }

            return lastHullVertexIndex + 1;
         }
         else
         {
            /*
             * Swap the candidate to be located right after the last vertex found in the list.
             */
            lastHullVertexIndex++;
            Collections.swap(vertices, lastHullVertexIndex, candidateIndex);

            if (visualizer != null)
            {

               visualizer.setPointCloud(vertices, numberOfVertices);
               visualizer.setLastHullVertexIndex(lastHullVertexIndex);
               visualizer.setPerimeter(vertices, lastHullVertexIndex + 1);
               visualizer.updateViz(AlgorithmPhase.UPDATING_PERIMETER);
            }
         }
      }

      if (visualizer != null)
      {
         visualizer.setPerimeter(vertices, numberOfVertices);
         visualizer.updateViz(AlgorithmPhase.RAN_THROUGH_ALL);
      }

      return numberOfVertices;
   }

   private static boolean isCandidatePointCollinearInOppositeDirectionWithTheFirstPoint(Point2DReadOnly lastHullVertex,
                                                                                        Point2DReadOnly previousHullVertex,
                                                                                        Point2DReadOnly candidateVertex,
                                                                                        double epsilon)
   {
      // get the normalized direction of the previous edge.
      double candidateDeltaX = lastHullVertex.getX() - previousHullVertex.getX();
      double candidateDeltaY = lastHullVertex.getY() - previousHullVertex.getY();
      double candidateNorm = EuclidCoreTools.norm(candidateDeltaX, candidateDeltaY);
      double candidateDirectionX = candidateDeltaX / candidateNorm;
      double candidateDirectionY = candidateDeltaY / candidateNorm;

      // Check if this new candidate point is collinear and in the opposite direction
      double vertexDeltaX = candidateVertex.getX() - lastHullVertex.getX();
      double vertexDeltaY = candidateVertex.getY() - lastHullVertex.getY();
      double vertexDeltaNorm = EuclidCoreTools.norm(vertexDeltaX, vertexDeltaY);

      Vector2D incomingEdge = new Vector2D();
      incomingEdge.sub(previousHullVertex, lastHullVertex);
      Vector2D outgoingEdge = new Vector2D();
      outgoingEdge.sub(candidateVertex, lastHullVertex);

      double x1 = previousHullVertex.getX() - lastHullVertex.getX();
      double y1 = previousHullVertex.getY() - lastHullVertex.getY();
      double x2 = candidateVertex.getX() - lastHullVertex.getX();
      double y2 = candidateVertex.getY() - lastHullVertex.getY();
      double angle = TupleTools.angle(x1,  y1, x2, y2);

      double dotProduct = dot(vertexDeltaX, vertexDeltaY, candidateDirectionX, candidateDirectionY);

      boolean badfromAngle = MathTools.epsilonEquals(angle, 0.0, epsilon);
      boolean badFromDotProduct = MathTools.epsilonEquals(dotProduct, -vertexDeltaNorm, epsilon);

      double sinTheta = x1 * y2 - y1 * x2;
      double cosTheta = x1 * x2 + y1 * y2;

      boolean badFromSin = MathTools.epsilonEquals(sinTheta, 0.0, epsilon) && cosTheta > -epsilon;

      if (badfromAngle != badFromDotProduct || badfromAngle != badFromSin)
         throw new RuntimeException("All three methods should be equivalent.");


      return badFromDotProduct || badFromSin || badfromAngle;
   }

   private static double dot(double deltaX1, double deltaY1, double deltaX2, double deltaY2)
   {
      return deltaX1 * deltaX2 + deltaY1 * deltaY2;
   }



   public static int findMinXMaxYVertexIndex(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return -1;

      int minXMaxYIndex = 0;
      Point2DReadOnly minXMaxY = vertices.get(minXMaxYIndex);

      for (int vertexIndex = 1; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Point2DReadOnly candidate = vertices.get(vertexIndex);

         if (candidate.getX() < minXMaxY.getX())
         {
            minXMaxYIndex = vertexIndex;
            minXMaxY = candidate;
         }
         else if (candidate.getX() == minXMaxY.getX() && candidate.getY() > minXMaxY.getY())
         {
            minXMaxYIndex = vertexIndex;
            minXMaxY = candidate;
         }
      }

      return minXMaxYIndex;
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }

   private static void assertPointsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point3D> points, double epsilon)
   {
      for (Point3D point : points)
      {
         double distance = polygonToCheck.signedDistance(new Point2D(point));

         assertTrue(distance < epsilon, "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(polygonToCheck.isPointInside(new Point2D(point), epsilon), "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private static void assertPointsInside(GiftWrappingVisualizer visualizer, ConvexPolygon2DReadOnly polygonToCheck, List<Point3D> points, double epsilon)
   {
      for (Point3D point : points)
      {
         double distanece = polygonToCheck.signedDistance(new Point2D(point));

         if (distanece < epsilon && polygonToCheck.isPointInside(new Point2D(point), epsilon))
         {
            visualizer.setTestPoint(new Point2D(point));
         }
         else
         {
            visualizer.setFailedPoint(new Point2D(point), distanece);
         }
         visualizer.updateViz(AlgorithmPhase.CHECKING_INTERIOR);
      }
   }

   private static void assertPoint2DsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point2D> points, double epsilon)
   {
      for (Point2D point : points)
      {
         double distance = polygonToCheck.signedDistance(new Point2D(point));

         assertTrue(polygonToCheck.isPointInside(point, epsilon), "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(distance < epsilon, "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private static void assertPoint2DsInside(GiftWrappingVisualizer visualizer, ConvexPolygon2DReadOnly polygonToCheck, List<Point2D> points, double epsilon)
   {
      if (visualizer == null)
         return;

      for (Point2D point : points)
      {
         double distanece = polygonToCheck.signedDistance(new Point2D(point));

         if (distanece < epsilon && polygonToCheck.isPointInside(new Point2D(point), epsilon))
         {
            visualizer.setTestPoint(new Point2D(point));
         }
         else
         {
            visualizer.setFailedPoint(new Point2D(point), distanece);
         }
         visualizer.updateViz(AlgorithmPhase.CHECKING_INTERIOR);
      }
   }

   private enum AlgorithmPhase
   {CHECKING_ALL_POINTS, UPDATING_CANDIDATE, UPDATING_QUERY, REJECTING_QUERY, REJECTING_CANDIDATE, FINISHED_LOOP, REMOVED_ALL, RAN_THROUGH_ALL, UPDATING_PERIMETER, CHECKING_INTERIOR, CHECKING_FOR_TOO_CLOSE_START, CHECKING_FOR_NONCOLLINEAR_START}

   private enum QueryRejection
   {TO_THE_RIGHT, TOO_CLOSE, COLLINEAR}


   private static class GiftWrappingVisualizer implements SCS2YoGraphicHolder
   {
      private final SimulationConstructionSet2 scs2;
      private final YoRegistry registry = new YoRegistry("GiftWrappingVisualizer");

      private final YoEnum<AlgorithmPhase> algorithmPhase = new YoEnum<>("algorithmPhase", registry, AlgorithmPhase.class);
      private final YoEnum<QueryRejection> queryRejection = new YoEnum<>("QueryRejection", registry, QueryRejection.class, true);

      private final List<YoFramePoint2D> pointCloud = new ArrayList<>();
      private final List<YoFramePoint2D> perimeterPoints = new ArrayList<>();
      private final List<YoFrameLineSegment2D> perimeterSegments = new ArrayList<>();

      private final YoInteger numberOfVertices = new YoInteger("numberOfVertices", registry);
      private final YoInteger lastHullVertexIndex = new YoInteger("lastHullVertexIndex", registry);
      private final YoInteger candidateIndex = new YoInteger("candidateIndex", registry);
      private final YoInteger queryIndex = new YoInteger("queryIndex", registry);
      private final YoFramePoint2D candidatePoint = new YoFramePoint2D("candidatePoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint2D queryPoint = new YoFramePoint2D("queryPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint2D testPoint = new YoFramePoint2D("testPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint2D failedPoint = new YoFramePoint2D("failedPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoDouble failedDistance = new YoDouble("failedDistance", registry);
      private final YoFramePoint2D lastHullPoint = new YoFramePoint2D("lastHullPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameLineSegment2D candidateSegment = new YoFrameLineSegment2D("candidateSegment", ReferenceFrame.getWorldFrame(), registry);

      private final YoFrameConvexPolygon2D convexPolygon2D;

      public GiftWrappingVisualizer(int numberOfPoints)
      {
         convexPolygon2D = new YoFrameConvexPolygon2D("convexPolygon2D", "", ReferenceFrame.getWorldFrame(), numberOfPoints, registry);
         scs2 = new SimulationConstructionSet2("GiftWrappingVisualizer");

         candidatePoint.setToNaN();
         queryPoint.setToNaN();
         lastHullPoint.setToNaN();
         candidateSegment.setToNaN();
         failedPoint.setToNaN();
         testPoint.setToNaN();

         for (int i = 0; i < numberOfPoints; i++)
         {
            YoFramePoint2D point = new YoFramePoint2D("pointCloud" + i, ReferenceFrame.getWorldFrame(), registry);
            YoFramePoint2D perimeterPoint = new YoFramePoint2D("perimeterPoint" + i, ReferenceFrame.getWorldFrame(), registry);
            point.setToNaN();
            perimeterPoint.setToNaN();
            pointCloud.add(point);
            perimeterPoints.add(perimeterPoint);
         }

         for (int i = 0; i < numberOfPoints - 1; i++)
         {
            YoFrameLineSegment2D segment = new YoFrameLineSegment2D("perimeterSegment" + i, ReferenceFrame.getWorldFrame(), registry);
            segment.setToNaN();
            perimeterSegments.add(segment);
         }

         scs2.addRegistry(registry);
         scs2.addYoGraphic(getSCS2YoGraphics());

         scs2.startSimulationThread();
      }

      public void setLastHullVertexIndex(int lastHullVertexIndex)
      {
         this.lastHullVertexIndex.set(lastHullVertexIndex);
      }

      public void setQueryRejection(QueryRejection queryRejection)
      {
         this.queryRejection.set(queryRejection);
      }

      public void setPointCloud(List<? extends Point2DReadOnly> points, int numberOfIndices)
      {
         this.numberOfVertices.set(numberOfIndices);
         //         convexPolygon2D.set(Vertex2DSupplier.asVertex2DSupplier(points));
         for (int i = 0; i < pointCloud.size(); i++)
         {
            if (i < points.size())
            {
               pointCloud.get(i).set(points.get(i));
            }
            else
            {
               pointCloud.get(i).setToNaN();
            }
         }
      }

      public void updateViz(AlgorithmPhase algorithmPhase)
      {
         this.algorithmPhase.set(algorithmPhase);
         scs2.simulateNow(1);
         ThreadTools.sleep(10);
      }

      public void setPerimeter(List<? extends Point2DReadOnly> points, int pointsOnPerimeter)
      {
         perimeterSegments.forEach(LineSegment2DBasics::setToNaN);
         perimeterPoints.forEach(Point2DBasics::setToNaN);
         queryPoint.setToNaN();
         candidatePoint.setToNaN();
         candidateSegment.setToNaN();

         for (int i = 0; i < Math.min(perimeterPoints.size(), pointsOnPerimeter); i++)
         {
            perimeterPoints.get(i).set(points.get(i));
            if (i > 0)
            {
               perimeterSegments.get(i - 1).set(perimeterPoints.get(i - 1), perimeterPoints.get(i));
            }
         }
      }

      public void setCandidate(Point2DReadOnly lastPointOnHull, Point2DReadOnly candidate, int candidateIndex)
      {
         queryRejection.set(null);
         lastHullPoint.set(lastPointOnHull);
         candidatePoint.set(candidate);
         candidateSegment.set(candidate, lastPointOnHull);
         queryPoint.setToNaN();
         this.candidateIndex.set(candidateIndex);
      }

      public void setQueryPoint(Point2DReadOnly queryPoint, int queryIndex)
      {
         this.queryPoint.set(queryPoint);
         this.queryIndex.set(queryIndex);
      }

      public void setTestPoint(Point2DReadOnly testPoint)
      {
         failedPoint.setToNaN();
         failedDistance.set(Double.NaN);
         this.testPoint.set(testPoint);
      }

      public void setFailedPoint(Point2DReadOnly failedPoint, double failedDistance)
      {
         testPoint.setToNaN();
         this.failedPoint.set(failedPoint);
         this.failedDistance.set(failedDistance);
      }

      @Override
      public YoGraphicDefinition getSCS2YoGraphics()
      {
         YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
         for (int i = 0; i < pointCloud.size(); i++)
         {
            YoGraphicPoint2DDefinition pointCloudVid = newYoGraphicPoint2D("PointCloudPoint " + i,
                                                                           pointCloud.get(i),
                                                                           0.004,
                                                                           ColorDefinitions.DarkViolet(),
                                                                           YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE);

            YoGraphicPoint2DDefinition perimeterPointViz = newYoGraphicPoint2D("perimeterPointViz " + i,
                                                                               perimeterPoints.get(i),
                                                                               0.004,
                                                                               ColorDefinitions.DarkViolet(),
                                                                               YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_FILLED);

            group.addChild(pointCloudVid);
            group.addChild(perimeterPointViz);
         }

         for (int i = 0; i < perimeterSegments.size(); i++)
         {
            YoGraphicLine2DDefinition segmentViz = newYoGraphicLineSegment2DDefinition("perimeterSegment" + i,
                                                                                       perimeterSegments.get(i),
                                                                                       ColorDefinitions.DarkViolet());
            group.addChild(segmentViz);
         }

         YoGraphicPoint2DDefinition candidatePointViz = newYoGraphicPoint2D("CandidatePoint",
                                                                            candidatePoint,
                                                                            0.006,
                                                                            ColorDefinitions.Green(),
                                                                            YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_CROSS);
         YoGraphicLine2DDefinition candidateSegmentViz = newYoGraphicLineSegment2DDefinition("CandidateSegment", candidateSegment, ColorDefinitions.Green());
         YoGraphicPoint2DDefinition lastHullPointViz = newYoGraphicPoint2D("lastHullPoint",
                                                                           lastHullPoint,
                                                                           0.006,
                                                                           ColorDefinitions.DarkViolet(),
                                                                           YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_FILLED);

         YoGraphicPoint2DDefinition queryPointViz = newYoGraphicPoint2D("queryPoint",
                                                                        queryPoint,
                                                                        0.006,
                                                                        ColorDefinitions.Blue(),
                                                                        YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_CROSS);

         YoGraphicPoint2DDefinition testPointViz = newYoGraphicPoint2D("testPoint", testPoint, 0.006, ColorDefinitions.Green(), DefaultPoint2DGraphic.CIRCLE);
         YoGraphicPoint2DDefinition failedPointViz = newYoGraphicPoint2D("failedPoint",
                                                                         failedPoint,
                                                                         0.006,
                                                                         ColorDefinitions.Red(),
                                                                         DefaultPoint2DGraphic.CIRCLE);

         YoGraphicPolygon2DDefinition convexPolygonViz = newYoGraphicPolygon2D("convexPolygon", convexPolygon2D, ColorDefinitions.Red());

         group.addChild(candidatePointViz);
         group.addChild(candidateSegmentViz);
         group.addChild(lastHullPointViz);
         group.addChild(queryPointViz);
         //         group.addChild(convexPolygonViz);
         group.addChild(testPointViz);
         group.addChild(failedPointViz);

         return group;
      }
   }
}
