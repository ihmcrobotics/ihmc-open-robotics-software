package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.*;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

public class ConvexPolygon2DTest
{

   static final double EPSILON = 1.0e-7;

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
      int iters = 1;
      for (int iter = 0; iter < iters; iter++)
      {
         // get a polygon transformed to a random location.
         RigidBodyTransform transform = new RigidBodyTransform();
         int gridXIndex = -50;
         int gridYIndex = -31;
         int yawIndex = 19;
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
         ConvexPolygon2D croppedFootholdOf3D = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
         List<Point2D> sortedPoints = footPointsInEnvironment.stream().map(Point2D::new).collect(Collectors.toList());
         ConvexPolygon2D croppedFootholdOf2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(sortedPoints));
         int numberOfPoints = inPlaceGiftWrapConvexHull2D(sortedPoints, sortedPoints.size());
         ThreadTools.sleepForever();
         assertEquals(croppedFootholdOf2D.getNumberOfVertices(), croppedFootholdOf3D.getNumberOfVertices());
         assertEquals(numberOfPoints, croppedFootholdOf2D.getNumberOfVertices());
         assertPointsInside(croppedFootholdOf3D, footPointsInEnvironment);

         List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                               {
                                                                                  Point3D transformedPoint = new Point3D(point);
                                                                                  transformedPoint.applyInverseTransform(transform);
                                                                                  return new Point2D(transformedPoint);
                                                                               }).toList();


         // Transform crop, and make sure all the transformed points are inside
         //         croppedFoothold.applyInverseTransform(transform, false);
         //         assertPoint2DsInside(croppedFoothold, footPointsInFoot);


         // Create a polygon around all the transformed points, and make sure they're inside
         ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
         assertPoint2DsInside(polygon2D, footPointsInFoot);
      }

      ThreadTools.sleepForever();
   }

   public static int inPlaceGiftWrapConvexHull2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {

      GiftWrappingVisualizer visualizer = new GiftWrappingVisualizer(numberOfVertices);
      visualizer.setPointCloud(vertices);

      if (numberOfVertices == 0)
         return 0;

      checkNumberOfVertices(vertices, numberOfVertices);

      /*
       * Set the first vertex to be the one with the lowest x-coordinate. If the lowest x-coordinate
       * exists in more than one vertex in the list, the vertex with the highest y-coordinate out of the
       * candidates is chosen.
       */
      Collections.swap(vertices, findMinXMaxYVertexIndex(vertices, numberOfVertices), 0);
      Point2DReadOnly firstVertex = vertices.get(0);

      // Idiot check to verify that it is indeed the lowest x-coordinate.
      for (int i = 1; i < vertices.size(); i++)
      {
         if (firstVertex.getX() > vertices.get(i).getX())
            throw new RuntimeException("Crap");
      }

      // This is the last index that belongs to the convex polygon that has been found so far.
      int lastHullVertexIndex = 0;

      while (lastHullVertexIndex < numberOfVertices - 1)
      {
         Point2DReadOnly lastHullVertex = vertices.get(lastHullVertexIndex);
         visualizer.setLastHullVertexIndex(lastHullVertexIndex);

         // Index to keep track of the potential next vertex of the polygon.
         int candidateIndex = lastHullVertexIndex + 1;
         Point2DReadOnly candidateVertex = vertices.get(candidateIndex);

         while (candidateVertex.epsilonEquals(lastHullVertex, EPSILON))
         { // Remove any duplicate vertices between here and the end of the list. We do this by swapping this vertex to the last of the list.
            Collections.swap(vertices, candidateIndex, --numberOfVertices);
            candidateVertex = vertices.get(candidateIndex);

            if (numberOfVertices == 1)
            {
               visualizer.setPerimeter(vertices, numberOfVertices);
               visualizer.updateViz(AlgorithmPhase.REMOVED_ALL);

               return numberOfVertices;
            }
         }

         visualizer.setPerimeter(vertices, lastHullVertexIndex);
         visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
         visualizer.updateViz(AlgorithmPhase.UPDATING_PERIMETER);


         // Loop through all the other vertices. Create a line segment starting at the last hull vertex, and ending at the candidate vertex.  If all the other
         // points are to the
         for (int vertexIndex = lastHullVertexIndex + 2; vertexIndex <= numberOfVertices;)
         {
            int wrappedIndex = EuclidCoreTools.wrap(vertexIndex, numberOfVertices);
            Point2DReadOnly vertex = vertices.get(wrappedIndex);

            visualizer.setQueryPoint(vertex, wrappedIndex);
            visualizer.updateViz(AlgorithmPhase.UDPATING_QUERY);

            if (vertex.epsilonEquals(candidateVertex, EPSILON) || wrappedIndex != 0 && vertex.epsilonEquals(firstVertex, EPSILON))
            { // Remove duplicate vertices
               Collections.swap(vertices, wrappedIndex, --numberOfVertices);
               /*
                * Restart iteration without incrementing the vertexIndex, so the wrappedIndex can be updated
                * properly as the numberOfVertices just changed.
                */
               continue;
            }

            if (isPoint2DOnLeftSideOfLine2D(vertex, lastHullVertex, candidateVertex))
            { // vertex is located outside => candidateVertex is not the next polygon vertex, vertex might be though.
               candidateIndex = wrappedIndex;
               candidateVertex = vertex;

               visualizer.setCandidate(lastHullVertex, candidateVertex, candidateIndex);
               visualizer.updateViz(AlgorithmPhase.UDPATING_CANDIDATE);
            }

            vertexIndex++;
         }

         if (candidateIndex == 0)
         {
            /*
             * Got back to the first vertex of the polygon: we're done and the size of the polygon is defined by
             * the index of the last vertex found.
             */
            visualizer.setPerimeter(vertices, lastHullVertexIndex + 1);
            visualizer.updateViz(AlgorithmPhase.FINISHED_LOOP);

            return lastHullVertexIndex + 1;
         }
         else
         {
            /*
             * Swap the candidate to be located right after the last vertex found in the list.
             */
            lastHullVertexIndex++;
            Collections.swap(vertices, lastHullVertexIndex, candidateIndex);

            visualizer.setLastHullVertexIndex(lastHullVertexIndex);
            visualizer.setPerimeter(vertices, lastHullVertexIndex);
            visualizer.updateViz(AlgorithmPhase.UPDATING_PERIMETER);

            // doing an idiot check
            int otherIndex = EuclidCoreTools.wrap(lastHullVertexIndex + 1, numberOfVertices);
            while (otherIndex != lastHullVertexIndex - 1)
            {
               if (isPoint2DOnLeftSideOfLine2D(vertices.get(otherIndex), vertices.get(lastHullVertexIndex - 1), vertices.get(lastHullVertexIndex)))
               { // vertex is located outside => candidateVertex is not the next polygon vertex, vertex might be though.
                  LogTools.info("This isn't good, there's a point on the other side of the polygon.");
               }

               otherIndex = EuclidCoreTools.wrap(otherIndex + 1, numberOfVertices);
            }
         }
      }


      visualizer.setPerimeter(vertices, numberOfVertices);
      visualizer.updateViz(AlgorithmPhase.RAN_THROUGH_ALL);

      return numberOfVertices;
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

   @Test
   public void testTerriblePoints()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double snapAreaResolution = 0.2;
      int iters = 100000;
      for (int iter = 0; iter < iters; iter++)
      {
         // get a polygon transformed to a random location.
         RigidBodyTransform transform = new RigidBodyTransform();
         int gridXIndex = RandomNumbers.nextInt(random, -100, 100);
         int gridYIndex = RandomNumbers.nextInt(random, -100, 100);
         int yawIndex = RandomNumbers.nextInt(random, 0, LatticePoint.yawDivisions);
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

         List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                               {
                                                                                  Point3D transformedPoint = new Point3D(point);
                                                                                  transformedPoint.applyInverseTransform(transform);
                                                                                  return new Point2D(transformedPoint);
                                                                               }).toList();

         // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
         //         ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
         //         assertPointsInside(croppedFoothold, footPointsInEnvironment);

         // Transform crop, and make sure all the transformed points are inside
         //         croppedFoothold.applyInverseTransform(transform, false);
         //         assertPoint2DsInside(croppedFoothold, footPointsInFoot);


         // Create a polygon around all teh transformed points, and make sure they're inside
         ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
         assertPoint2DsInside(polygon2D, footPointsInFoot);
      }
   }


   private static void assertPointsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point3D> points)
   {
      for (Point3D point : points)
      {
         double distance = polygonToCheck.signedDistance(new Point2D(point));
         assertTrue(distance < 1e-3, "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(polygonToCheck.isPointInside(new Point2D(point)), "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private static void assertPoint2DsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point2D> points)
   {
      for (Point2D point : points)
      {
         double distance = polygonToCheck.signedDistance(point);
         assertTrue(polygonToCheck.isPointInside(point, 1e-3), "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(distance < 1e-3, "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private enum AlgorithmPhase
   {CHECKING_ALL_POINTS, UDPATING_CANDIDATE, UDPATING_QUERY, FINISHED_LOOP, REMOVED_ALL, RAN_THROUGH_ALL, UPDATING_PERIMETER}
   private static class GiftWrappingVisualizer implements SCS2YoGraphicHolder
   {
      private final SimulationConstructionSet2 scs2;
      private final YoRegistry registry = new YoRegistry("GiftWrappingVisualizer");

      private final YoEnum<AlgorithmPhase> algorithmPhase = new YoEnum<>("algorithmPhase", registry, AlgorithmPhase.class);

      private final List<YoFramePoint2D> pointCloud = new ArrayList<>();
      private final List<YoFramePoint2D> perimeterPoints = new ArrayList<>();
      private final List<YoFrameLineSegment2D> perimeterSegments = new ArrayList<>();

      private final YoInteger lastHullVertexIndex = new YoInteger("lastHullVertexIndex", registry);
      private final YoInteger candidateIndex = new YoInteger("candidateIndex", registry);
      private final YoInteger queryIndex = new YoInteger("queryIndex", registry);
      private final YoFramePoint2D candidatePoint = new YoFramePoint2D("candidatePoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint2D queryPoint = new YoFramePoint2D("queryPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint2D lastHullPoint = new YoFramePoint2D("lastHullPoint", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameLineSegment2D candidateSegment = new YoFrameLineSegment2D("candidateSegment", ReferenceFrame.getWorldFrame(), registry);

      public GiftWrappingVisualizer(int numberOfPoints)
      {
         scs2 = new SimulationConstructionSet2("GiftWrappingVisualizer");

         candidatePoint.setToNaN();
         queryPoint.setToNaN();
         lastHullPoint.setToNaN();
         candidateSegment.setToNaN();

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

      public void setPointCloud(List<? extends Point2DReadOnly> points)
      {
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
         ThreadTools.sleep(100);
      }

      public void setPerimeter(List<? extends Point2DReadOnly> points, int pointsOnPerimeter)
      {
         perimeterSegments.forEach(LineSegment2DBasics::setToNaN);

         for (int i = 0; i < perimeterPoints.size(); i++)
         {
            if (i < pointsOnPerimeter)
            {
               perimeterPoints.get(i).set(points.get(i));
               if (i > 0)
               {
                  perimeterSegments.get(i - 1).set(perimeterPoints.get(i - 1), perimeterPoints.get(i));
               }
            }
            else
            {
               perimeterPoints.get(i).setToNaN();
            }
         }
      }

      public void setCandidate(Point2DReadOnly lastPointOnHull, Point2DReadOnly candidate, int candidateIndex)
      {
         lastHullPoint.set(lastPointOnHull);
         candidatePoint.set(candidate);
         candidateSegment.set(candidate, lastPointOnHull);
         this.candidateIndex.set(candidateIndex);
      }

      public void setQueryPoint(Point2DReadOnly queryPoint, int queryIndex)
      {
         this.queryPoint.set(queryPoint);
         this.queryIndex.set(queryIndex);
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
         YoGraphicLine2DDefinition candidateSegmentViz = newYoGraphicLineSegment2DDefinition("CandidateSegment",
                                                                            candidateSegment,
                                                                            ColorDefinitions.Green());
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
         group.addChild(candidatePointViz);
         group.addChild(candidateSegmentViz);
         group.addChild(lastHullPointViz);
         group.addChild(queryPointViz);

         return group;
      }
   }

}
