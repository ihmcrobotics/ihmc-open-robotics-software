package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.triangulate.ConformingDelaunayTriangulationBuilder;
import com.vividsolutions.jts.triangulate.ConstraintEnforcementException;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeSubdivision;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

/**
 * Computes the concave hull of a 2D point cloud based on the paper
 * <a href="http://worboys.org/publications/Duckham%20Worboys%20Galton%20PatRec%20final%202008.pdf">
 *  Efficient generation of simple polygons for characterizing the shape of a set of points in the plane </a>.
 *  <p>
 *  To summarize, it is based on the Delaunay triangulation.
 *  The only parameter for this algorithm is the maximum edge length the concave hull.
 *  The resulting concave hull is simple:
 *  <li> no self intersections,
 *  <li> no holes,
 *  <li> no detached polygons.
 *  
 * @author Sylvain
 *
 */
public abstract class SimpleConcaveHullFactory
{
   private static final boolean VERBOSE = false;
   private static final boolean REPORT_TIME = false;

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2D> pointCloud2d, ConcaveHullFactoryParameters parameters)
   {
      return createConcaveHullCollection(pointCloud2d, null, parameters);
   }

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2D> pointCloud2d, List<LineSegment2D> lineConstraints, ConcaveHullFactoryParameters parameters)
   {
      if (pointCloud2d.size() <= 3)
         return new ConcaveHullCollection(pointCloud2d);

      return createConcaveHull(pointCloud2d, lineConstraints, parameters).getConcaveHullCollection();
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2D> pointCloud2d, ConcaveHullFactoryParameters parameters)
   {
      return createConcaveHull(pointCloud2d, null, parameters);
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2D> pointCloud2d, List<LineSegment2D> lineConstraints, ConcaveHullFactoryParameters parameters)
   {
      if (pointCloud2d.size() <= 3)
         return null;

      MultiPoint sites = filterAndCreateMultiPoint(pointCloud2d, lineConstraints, 0.01);
      MultiLineString constraintSegments = createMultiLineString(lineConstraints);
      ConcaveHullFactoryResult result = new ConcaveHullFactoryResult();
      ConcaveHullVariables initialVariables = initializeTriangulation(sites, constraintSegments, result);
      List<ConcaveHullVariables> variablesList = computeConcaveHullBorderEdgesRecursive(parameters, initialVariables);
      result.intermediateVariables.addAll(variablesList);

      for (ConcaveHullVariables variables : result.intermediateVariables)
      {
         ConcaveHull concaveHull = computeConcaveHull(variables.getOrderedBorderEdges());
         if (concaveHull != null)
         {
            concaveHull.ensureClockwiseOrdering();
            result.concaveHullCollection.add(concaveHull);
         }
      }

      return result;
   }

   public static MultiPoint filterAndCreateMultiPoint(List<Point2D> pointCloud2d, List<LineSegment2D> lineConstraints, double tolerance)
   {
      List<Point2D> filteredPointCloud2d = new ArrayList<>();

      for (Point2D point : pointCloud2d)
      {
         if (!isTooCloseToConstraintSegments(point, lineConstraints, tolerance))
            filteredPointCloud2d.add(point);
      }
      return createMultiPoint(filteredPointCloud2d);
   }

   public static boolean isTooCloseToConstraintSegments(Point2D point, List<LineSegment2D> lineConstraints, double tolerance)
   {
      for (LineSegment2D lineConstraint : lineConstraints)
      {
         if (lineConstraint.distance(point) < tolerance)
            return true;
      }
      return false;
   }

   public static MultiPoint createMultiPoint(List<Point2D> pointCloud2d)
   {
      Coordinate[] coordinates = new Coordinate[pointCloud2d.size()];

      for (int i = 0; i < pointCloud2d.size(); i++)
      {
         Point2D point2d = pointCloud2d.get(i);
         coordinates[i] = new Coordinate(point2d.getX(), point2d.getY());
      }

      return new GeometryFactory().createMultiPoint(coordinates);
   }

   public static MultiLineString createMultiLineString(List<LineSegment2D> lineSegments)
   {
      if (lineSegments == null)
         return null;

      List<LineString> lineStrings = lineSegments.stream()
                                                 .map(SimpleConcaveHullFactory::createLineString)
                                                 .collect(Collectors.toList());

      GeometryFactory geometryFactory = new GeometryFactory();

      // Try to merge lineStrings
      for (int i = 0; i < lineStrings.size(); i++)
      {
         LineString firstLineString = lineStrings.get(i);
         List<Coordinate> firstCoordinates = new ArrayList<>(Arrays.asList(firstLineString.getCoordinates()));

         for (int j = lineStrings.size() - 1; j >= i + 1; j--)
         {
            LineString secondLineString = lineStrings.get(j);
            List<Coordinate> secondCoordinates = new ArrayList<>(Arrays.asList(secondLineString.getCoordinates()));

            boolean concatenate = false;

            double tolerance = 1.0e-3;
            if (firstLineString.getEndPoint().equalsExact(secondLineString.getStartPoint(), tolerance))
            {
               concatenate = true;
            }
            else if (firstLineString.getEndPoint().equalsExact(secondLineString.getEndPoint(), tolerance))
            {
               concatenate = true;
               Collections.reverse(secondCoordinates);
            }
            else if (firstLineString.getStartPoint().equalsExact(secondLineString.getEndPoint(), tolerance))
            {
               concatenate = true;
               Collections.reverse(firstCoordinates);
               Collections.reverse(secondCoordinates);
            }
            else if (firstLineString.getStartPoint().equalsExact(secondLineString.getStartPoint(), tolerance))
            {
               concatenate = true;
               Collections.reverse(firstCoordinates);
            }

            if (concatenate)
            {
               lineStrings.remove(j);
               secondCoordinates.remove(0);
               firstCoordinates.addAll(secondCoordinates);
               firstLineString = geometryFactory.createLineString(firstCoordinates.toArray(new Coordinate[firstCoordinates.size()]));
               lineStrings.set(i, firstLineString);
            }
         }
      }

      return geometryFactory.createMultiLineString(lineStrings.toArray(new LineString[lineStrings.size()]));
   }

   public static LineString createLineString(LineSegment2D lineSegment)
   {
      Coordinate lineSegmentStart = new Coordinate(lineSegment.getFirstEndpointX(), lineSegment.getFirstEndpointY());
      Coordinate lineSegmentEnd = new Coordinate(lineSegment.getSecondEndpointX(), lineSegment.getSecondEndpointY());
      
      Coordinate[] endPoints = {lineSegmentStart, lineSegmentEnd};
      return new GeometryFactory().createLineString(endPoints);
   }

   private static ConcaveHull computeConcaveHull(List<QuadEdge> orderedBorderEdges)
   {
      List<Point2D> orderedConcaveHullVertices = orderedBorderEdges.stream()
                                                                   .map(QuadEdge::orig)
                                                                   .map(vertex -> new Point2D(vertex.getX(), vertex.getY()))
                                                                   .collect(Collectors.toList());
      return new ConcaveHull(orderedConcaveHullVertices);
   }

   /**
    * 
    * @param sites the point cloud from which a concave hull is to be computed.
    * @param constraintSegments 
    * @param concaveHullFactoryResult
    * @return
    */
   @SuppressWarnings("unchecked")
   private static ConcaveHullVariables initializeTriangulation(MultiPoint sites, MultiLineString constraintSegments, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      // NOTE: The DelaunayTriangulatorBuilder is 30% to 40% faster than the ConformingDelaunayTriangulationBuilder.
      ConformingDelaunayTriangulationBuilder conformingDelaunayTriangulationBuilder = new ConformingDelaunayTriangulationBuilder();
      conformingDelaunayTriangulationBuilder.setTolerance(10.0e-3);
      conformingDelaunayTriangulationBuilder.setSites(sites);
      QuadEdgeSubdivision subdivision;
      try
      {
         if (constraintSegments != null)
            conformingDelaunayTriangulationBuilder.setConstraints(constraintSegments);
         subdivision = conformingDelaunayTriangulationBuilder.getSubdivision();
      }
      catch (ConstraintEnforcementException e)
      { // Adding the line segments as constraints failed, removing them.
         if (VERBOSE)
            PrintTools.warn(SimpleConcaveHullFactory.class, "Delaunay triangulation failed, removing line segment constraints.");
         conformingDelaunayTriangulationBuilder.setConstraints(null);
         subdivision = conformingDelaunayTriangulationBuilder.getSubdivision();
      }
      // All the triangles resulting from the triangulation.
      List<QuadEdgeTriangle> allTriangles = concaveHullFactoryResult.allTriangles;
      allTriangles.addAll(QuadEdgeTriangle.createOn(subdivision));

      if (REPORT_TIME)
      {
         System.out.println("Triangulation took: " + Conversions.nanosecondsToSeconds(stopWatch.getNanoTime()) + " sec.");
      }

      return computeIntermediateVariables(allTriangles, constraintSegments);
   }

   public static ConcaveHullVariables computeIntermediateVariables(List<QuadEdgeTriangle> delaunayTriangles, MultiLineString constraintSegments)
   {
      ConcaveHullVariables variables = new ConcaveHullVariables();
      // Vertices of the concave hull
      Set<Vertex> borderVertices = variables.borderVertices;
      // Triangles with at least one edge that belongs to the concave hull.
      Set<QuadEdgeTriangle> borderTriangles = variables.borderTriangles;
      // The output of this method, the edges defining the concave hull
      Set<QuadEdge> borderEdges = variables.borderEdges;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = variables.sortedByLengthQueue;
      Set<QuadEdge> constraintEdges = variables.constraintEdges;

      QuadEdge firstBorderEdge = null;

      // Initialize the border triangles, edges, and vertices. The triangulation provides that information.
      for (QuadEdgeTriangle triangle : delaunayTriangles)
      {
         // Direct result from the triangulation
         if (triangle.isBorder())
         {
            borderTriangles.add(triangle);
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
               // Direct result from the triangulation
               if (triangle.isBorder(edgeIndex))
               {
                  QuadEdge borderEdge = triangle.getEdge(edgeIndex);
                  if (firstBorderEdge == null)
                     firstBorderEdge = borderEdge.getPrimary();

                  borderEdges.add(borderEdge);
                  borderVertices.add(borderEdge.orig());
                  borderVertices.add(borderEdge.dest());
                  sortedByLengthQueue.add(new ImmutablePair<>(borderEdge, triangle));
               }
            }
         }
      }

      if (constraintSegments != null)
      {
         for (QuadEdgeTriangle triangle : delaunayTriangles)
         {
            
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
               QuadEdge edge = triangle.getEdge(edgeIndex);
               if (isEdgeCollinearWithALineSemgentOfMultiLineString(edge, constraintSegments))
               {
                  constraintEdges.add(edge.getPrimary());
               }
            }
         }
      }

      List<QuadEdge> orderedBorderEdges = variables.orderedBorderEdges;
      orderedBorderEdges.add(firstBorderEdge);
      Vertex startVertex = firstBorderEdge.orig();
      Vertex currentDestVertex = firstBorderEdge.dest();
      QuadEdge previousEdge = firstBorderEdge;

      while (true)
      {
         QuadEdge currentEdge = null;
         QuadEdge currentIncidentEdge = previousEdge.dNext();

         while (currentIncidentEdge != previousEdge)
         {
            if (isBorderEdge(currentIncidentEdge, borderEdges))
            {
               currentEdge = currentIncidentEdge.sym();
               break;
            }
            currentIncidentEdge = currentIncidentEdge.dNext();
         }

         if (currentDestVertex.equals(startVertex))
            break;

         orderedBorderEdges.add(currentEdge);
         previousEdge = currentEdge;
         currentDestVertex = currentEdge.dest();
      }
      checkOrderedBorderEdgeListValid(orderedBorderEdges);

      // The triangulation is malformed and discontinuous
      if (orderedBorderEdges.size() < borderEdges.size())
         return null;
      else
         return variables;
   }

   private static boolean isEdgeCollinearWithALineSemgentOfMultiLineString(QuadEdge query, MultiLineString multiLineString)
   {
      for (int i = 0; i < multiLineString.getNumGeometries(); i++)
      {
         LineString lineString = (LineString) multiLineString.getGeometryN(i);
         if (isEdgeCollinearWithALineSemgentOfLineString(query, lineString))
            return true;
      }

      return false;
   }

   private static boolean isEdgeCollinearWithALineSemgentOfLineString(QuadEdge query, LineString lineString)
   {
      int finalIndex = lineString.isClosed() ? lineString.getNumPoints() : lineString.getNumPoints() - 1;

      for (int i = 0; i < finalIndex; i++)
      {
         Coordinate lineSegmentStart = lineString.getCoordinateN(i);
         Coordinate lineSegmentEnd = lineString.getCoordinateN((i + 1) % lineString.getNumPoints());

         if (isEdgeCollinearWithLineSegment(query, lineSegmentStart, lineSegmentEnd))
            return true;
      }

      return false;
   }

   private static boolean isEdgeCollinearWithLineSegment(QuadEdge query, Coordinate lineSegmentStart, Coordinate lineSegmentEnd)
   {
      Point2D firstPointOnLine1 = toPoint2d(query.orig());
      Point2D secondPointOnLine1 = toPoint2d(query.dest());
      Point2D firstPointOnLine2 = toPoint2d(lineSegmentStart);
      Point2D secondPointOnLine2 = toPoint2d(lineSegmentEnd);
      double angleEpsilon = Epsilons.ONE_MILLIONTH;
      double distanceEpsilon = Epsilons.ONE_TRILLIONTH;
      return EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEpsilon);
   }

   private static Point2D toPoint2d(Vertex vertex)
   {
      return new Point2D(vertex.getX(), vertex.getY());
   }

   private static Point2D toPoint2d(Coordinate coordinate)
   {
      return new Point2D(coordinate.x, coordinate.y);
   }

   /**
    * Computes the border edges {@link QuadEdge} that will define the concave hull.
    * This is an iterative process that starts a first guess of the concave hull, and then each iteration consists "breaking" edges that are too long according to the {@code edgeLengthThreshold}.
    * The algorithm is based on the <a href="https://en.wikipedia.org/wiki/Delaunay_triangulation"> Delaunay triangulation </a>.
    * @param edgeLengthThreshold maximum edge length the concave hull can have.
    * @param maxNumberOfIterations option to limit the maximum number of iterations of this algorithm.
    * @param removeAllTrianglesWithTwoBorderEdges when set to true, any triangle with two border edges with be removed regardless of the edges length. This tends to smoothen the resulting concave hull in general.
    * @param variables the set of variables pre-initialized used internally to find the border edges of the concave hull.
    * @return list of new intermediate variables containing the sets of edges defining the concave hull(s).
    */
   private static List<ConcaveHullVariables> computeConcaveHullBorderEdgesRecursive(ConcaveHullFactoryParameters parameters, ConcaveHullVariables variables)
   {
      return computeConcaveHullBorderEdgesRecursive(parameters, variables, new MutableInt(0));
   }

   private enum Case
   {
      KEEP_TRIANGLE,
      ONE_BORDER_EDGE_TWO_BORDER_VERTICES,
      TWO_BORDER_EDGES_THREE_BORDER_VERTICES,
      ONE_BORDER_EDGES_THREE_BORDER_VERTICES,
      THREE_BORDER_EDGES_THREE_BORDER_VERTICES,
      INTERSECTION_TRIANGLE;
   };

   @SuppressWarnings("unchecked")
   public static Case determineCase(Pair<QuadEdge, QuadEdgeTriangle> candidatePair, ConcaveHullFactoryParameters parameters, ConcaveHullVariables variables)
   {
      QuadEdgeComparator quadEdgeComparator = variables.quadEdgeComparator;
      Set<Vertex> borderVertices = variables.borderVertices;
      Set<QuadEdge> borderEdges = variables.borderEdges;
      Set<QuadEdgeTriangle> borderTriangles = variables.borderTriangles;
      Set<QuadEdge> constraintEdges = variables.constraintEdges;

      QuadEdge candidateEdge = candidatePair.getLeft();
      QuadEdgeTriangle candidateTriangle = candidatePair.getRight();
      double edgeLength = quadEdgeComparator.getEdgeLength(candidateEdge);
      boolean isEdgeTooLong = edgeLength >= parameters.getEdgeLengthThreshold();
      int numberOfBorderVertices = numberOfBorderVertices(candidateTriangle, borderVertices);
      int numberOfBorderEdges = numberOfBorderEdges(candidateTriangle, borderEdges);

      // Check if the triangle has a border that is a constraint.
      for (QuadEdge edge : candidateTriangle.getEdges())
      {
         if (isConstraintEdge(edge, constraintEdges) && isBorderEdge(edge, borderEdges))
            return Case.KEEP_TRIANGLE;
      }

      // Check if by removing the triangle, a constraint edge will become a border edge
      boolean forceRemovalToRevealConstraint = false;
      for (QuadEdge edge : candidateTriangle.getEdges())
      {
         if (isConstraintEdge(edge, constraintEdges))
         {
            forceRemovalToRevealConstraint = true;
         }
      }

      if (numberOfBorderVertices == 2)
      {
         if (numberOfBorderEdges != 1)
            throw new RuntimeException("Triangle should have one border edge, but has: " + numberOfBorderEdges);

         return isEdgeTooLong || forceRemovalToRevealConstraint ? Case.ONE_BORDER_EDGE_TWO_BORDER_VERTICES : Case.KEEP_TRIANGLE;
      }

      if (numberOfBorderEdges == 2)
      {
         if (numberOfBorderVertices != 3)
            throw new RuntimeException("Triangle should have three border vertices, but has: " + numberOfBorderVertices);

         if (parameters.doRemoveAllTrianglesWithTwoBorderEdges() || isEdgeTooLong)
         {
            // Here the triangle has only one edge inside the hull. If another border triangle shares the vertex opposite to this edge, the vertex is an intersection vertex.
            QuadEdge uniqueNonBorderEdge = Arrays.stream(candidateTriangle.getEdges()).filter(edge -> !isBorderEdge(edge, borderEdges)).findFirst().get();
            int vertexIndexOppositeToCandidateEdge = indexOfVertexOppositeToEdge(candidateTriangle.getEdgeIndex(uniqueNonBorderEdge));
            List<QuadEdgeTriangle> adjacentTrianglesToVertex = candidateTriangle.getTrianglesAdjacentToVertex(vertexIndexOppositeToCandidateEdge);

            boolean isIntersectionTriangle = adjacentTrianglesToVertex.stream()
                                                                      .filter(borderTriangles::contains)
                                                                      .filter(triangle -> triangle != candidateTriangle)
                                                                      .findAny().isPresent();
            if (isIntersectionTriangle)
               return parameters.isSplittingConcaveHullAllowed() ? Case.INTERSECTION_TRIANGLE : Case.KEEP_TRIANGLE;
            else
               return Case.TWO_BORDER_EDGES_THREE_BORDER_VERTICES;
         }
         else
         {
            return Case.KEEP_TRIANGLE;
         }
      }

      if (!parameters.isSplittingConcaveHullAllowed())
         return Case.KEEP_TRIANGLE;

      if (numberOfBorderEdges == 1)
         return Case.ONE_BORDER_EDGES_THREE_BORDER_VERTICES;
      else
         return Case.THREE_BORDER_EDGES_THREE_BORDER_VERTICES;
   }

   private static List<ConcaveHullVariables> computeConcaveHullBorderEdgesRecursive(ConcaveHullFactoryParameters parameters, List<ConcaveHullVariables> variables, MutableInt currentIteration)
   {
      List<ConcaveHullVariables> result = new ArrayList<>();
      for (ConcaveHullVariables hullVariables : variables)
      {
         currentIteration.increment();
         result.addAll(computeConcaveHullBorderEdgesRecursive(parameters, hullVariables, currentIteration));
      }
      return result;
   }

   private static List<ConcaveHullVariables> computeConcaveHullBorderEdgesRecursive(ConcaveHullFactoryParameters parameters, ConcaveHullVariables variables, MutableInt currentIteration)
   {
      if (currentIteration.intValue() >= parameters.getMaxNumberOfIterations())
      {
         if (VERBOSE)
            System.out.println("Reached max number of iterations");
         return Collections.singletonList(variables);
      }

      if (variables == null)
         return Collections.emptyList();

      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = variables.sortedByLengthQueue;

      List<Pair<QuadEdge, QuadEdgeTriangle>> bakup = new ArrayList<>();

      Case currentCase = Case.KEEP_TRIANGLE;
      Pair<QuadEdge, QuadEdgeTriangle> candidateEntry = null;

      while (!sortedByLengthQueue.isEmpty())
      {
         candidateEntry = sortedByLengthQueue.poll();
         currentCase = determineCase(candidateEntry, parameters, variables);
         if (currentCase == Case.KEEP_TRIANGLE)
         {
            // The entry's triangle is not removable this iteration, but it might later.
            // So put it in a backup that'll be emptied back in the main queue.
            // Not elegant, but couldn't figure out a way to navigate the queue and only remove a specific entry.
            // Note: the PriorityQueue's iterator is not sorted.
            bakup.add(candidateEntry);
            continue;
         }
         else
         {
            break;
         }
      }

      sortedByLengthQueue.addAll(bakup);
      bakup.clear();

      switch (currentCase)
      {
      case ONE_BORDER_EDGE_TWO_BORDER_VERTICES:
         removeTriangleWithOneBorderEdge(variables, candidateEntry);
         currentIteration.increment();
         return computeConcaveHullBorderEdgesRecursive(parameters, variables, currentIteration);
      case TWO_BORDER_EDGES_THREE_BORDER_VERTICES:
         removeTriangleWithTwoBorderEdges(variables, candidateEntry);
         currentIteration.increment();
         return computeConcaveHullBorderEdgesRecursive(parameters, variables, currentIteration);
      case ONE_BORDER_EDGES_THREE_BORDER_VERTICES:
      {
         List<ConcaveHullVariables> subVariablesList = removeTriangleAndDivideHull(variables, candidateEntry);
         return computeConcaveHullBorderEdgesRecursive(parameters, subVariablesList, currentIteration);
      }
      case INTERSECTION_TRIANGLE:
      {
         List<ConcaveHullVariables> subVariablesList = divideHullAtIntersectionTriangle(variables, candidateEntry);
         return computeConcaveHullBorderEdgesRecursive(parameters, subVariablesList, currentIteration);
      }
      case THREE_BORDER_EDGES_THREE_BORDER_VERTICES:
         return Collections.emptyList(); // FIXME
      case KEEP_TRIANGLE:
         if (VERBOSE)
            System.out.println("Done, number of iterations: " + currentIteration.intValue());
         return Collections.singletonList(variables);
      default:
         throw new RuntimeException("Unknown case: " + currentCase);
      }
   }

   private static void removeTriangleWithOneBorderEdge(ConcaveHullVariables variables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      QuadEdgeTriangle borderTriangleToRemove = entryToRemove.getRight();
      QuadEdge edgeToRemove = entryToRemove.getLeft();

      int indexOfTriangleEdgeToRemove = borderTriangleToRemove.getEdgeIndex(edgeToRemove);
      int indexAfterRemovedEdge = QuadEdgeTriangle.nextIndex(indexOfTriangleEdgeToRemove);
      int indexBeforeRemovedEdge = QuadEdgeTriangle.nextIndex(indexAfterRemovedEdge);

      Set<QuadEdgeTriangle> borderTriangles = variables.borderTriangles;
      Set<QuadEdge> borderEdges = variables.borderEdges;
      Set<Vertex> borderVertices = variables.borderVertices;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = variables.sortedByLengthQueue;

      // Remove the triangle and its edge
      borderTriangles.remove(borderTriangleToRemove);
      borderEdges.remove(edgeToRemove);
      borderEdges.remove(edgeToRemove.sym());

      // Get and add the two adjacent triangles
      QuadEdge newBorderEdgeAfterRemovedEdge = borderTriangleToRemove.getEdge(indexAfterRemovedEdge).sym();
      QuadEdgeTriangle newBorderTriangleAfterRemovedTriangle = (QuadEdgeTriangle) newBorderEdgeAfterRemovedEdge.getData();
      QuadEdge newBorderEdgeBeforeRemovedEdge = borderTriangleToRemove.getEdge(indexBeforeRemovedEdge).sym();
      QuadEdgeTriangle newBorderTriangleBeforeRemovedTriangle = (QuadEdgeTriangle) newBorderEdgeBeforeRemovedEdge.getData();

      borderTriangles.add(newBorderTriangleAfterRemovedTriangle);
      borderEdges.add(newBorderEdgeAfterRemovedEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdgeAfterRemovedEdge, newBorderTriangleAfterRemovedTriangle));

      borderTriangles.add(newBorderTriangleBeforeRemovedTriangle);
      borderEdges.add(newBorderEdgeBeforeRemovedEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdgeBeforeRemovedEdge, newBorderTriangleBeforeRemovedTriangle));

      // Add the vertex opposite of the removed edge. Its index is the same as beforeEdgeIndex
      borderVertices.add(borderTriangleToRemove.getVertex(indexBeforeRemovedEdge));

      List<QuadEdge> orderedBorderEdges = variables.orderedBorderEdges;
      replaceOneEdgeWithTwoInOrderedList(orderedBorderEdges, edgeToRemove, newBorderEdgeBeforeRemovedEdge, newBorderEdgeAfterRemovedEdge);
   }

   private static void removeTriangleWithTwoBorderEdges(ConcaveHullVariables variables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      QuadEdgeTriangle borderTriangleToRemove = entryToRemove.getRight();

      Set<QuadEdgeTriangle> borderTriangles = variables.borderTriangles;
      Set<QuadEdge> borderEdges = variables.borderEdges;
      Set<Vertex> borderVertices = variables.borderVertices;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = variables.sortedByLengthQueue;

      int newBorderEdgeIndex = -1;
      QuadEdge newBorderEdge = null;

      // Remove the triangle, its edges, and one vertex
      borderTriangles.remove(borderTriangleToRemove);
      for (int i = 0; i < 3; i++)
      {
         QuadEdge edge = borderTriangleToRemove.getEdge(i);
         if (!isBorderEdge(edge, borderEdges))
         {
            newBorderEdgeIndex = i;
            newBorderEdge = edge.sym(); // This edge becomes a border edge as we remove the triangle
            continue;
         }
         borderEdges.remove(edge);
         borderEdges.remove(edge.sym());
         sortedByLengthMap.remove(new ImmutablePair<>(edge, borderTriangleToRemove));
      }

      borderVertices.remove(borderTriangleToRemove.getVertex(indexOfVertexOppositeToEdge(newBorderEdgeIndex)));

      // Get and add the one adjacent triangle
      QuadEdgeTriangle newBorderTriangle = (QuadEdgeTriangle) newBorderEdge.getData();

      borderTriangles.add(newBorderTriangle);
      borderEdges.add(newBorderEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdge, newBorderTriangle));

      List<QuadEdge> orderedBorderEdges = variables.orderedBorderEdges;
      replaceTwoEdgesWithOneInOrderedList(orderedBorderEdges, borderTriangleToRemove.getEdge((newBorderEdgeIndex + 2) % 3), borderTriangleToRemove.getEdge((newBorderEdgeIndex + 1) % 3), newBorderEdge);
   }

   private static List<ConcaveHullVariables> removeTriangleAndDivideHull(ConcaveHullVariables variables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      QuadEdge edgeToRemove = entryToRemove.getLeft();
      QuadEdgeTriangle triangleToRemove = entryToRemove.getRight();
      Vertex intersectionVertex = triangleToRemove.getVertex(indexOfVertexOppositeToEdge(triangleToRemove.getEdgeIndex(edgeToRemove)));

      removeTriangleWithOneBorderEdge(variables, entryToRemove);

      return divideHullAtIntersectionVertex(variables, intersectionVertex);
   }

   private static List<ConcaveHullVariables> divideHullAtIntersectionTriangle(ConcaveHullVariables variables, Pair<QuadEdge, QuadEdgeTriangle> entryWithIntersectionTriangle)
   {
      Set<QuadEdge> borderEdges = variables.borderEdges;

      QuadEdgeTriangle intersectionTriangle = entryWithIntersectionTriangle.getRight();
      QuadEdge uniqueNonBorderEdge = Arrays.stream(intersectionTriangle.getEdges()).filter(edge -> !isBorderEdge(edge, borderEdges)).findAny().get();
      Vertex intersectionVertex = intersectionTriangle.getVertex(indexOfVertexOppositeToEdge(intersectionTriangle.getEdgeIndex(uniqueNonBorderEdge)));
      return divideHullAtIntersectionVertex(variables, intersectionVertex);
   }

   private static List<ConcaveHullVariables> divideHullAtIntersectionVertex(ConcaveHullVariables variables, Vertex intersectionVertex)
   {
      List<QuadEdge> orderedBorderEdges = variables.orderedBorderEdges;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = variables.sortedByLengthQueue;
      List<ConcaveHullVariables> dividedHullVariables = new ArrayList<>();

      int startIndex = IntStream.range(0, orderedBorderEdges.size())
                                .filter(i -> orderedBorderEdges.get(i).orig() == intersectionVertex)
                                .findAny()
                                .getAsInt();
      int previousEndIndex = ListWrappingIndexTools.previous(startIndex, orderedBorderEdges);

      do
      {
         int subHullStartIndex = ListWrappingIndexTools.next(previousEndIndex, orderedBorderEdges);
         int subHullEndIndex = ListWrappingIndexTools.next(subHullStartIndex, orderedBorderEdges);

         int count = 0;
         while (!doesVertexBelongToQuadEdge(intersectionVertex, orderedBorderEdges.get(subHullEndIndex)))
         {
            subHullEndIndex = ListWrappingIndexTools.next(subHullEndIndex, orderedBorderEdges);
            if (count++ >= orderedBorderEdges.size())
               throw new RuntimeException("Wrapped around the hull without finding the subHullEndIndex");
         }
         previousEndIndex = subHullEndIndex;

         if (ListWrappingIndexTools.subLengthInclusive(subHullStartIndex, subHullEndIndex, orderedBorderEdges) <= 3)
            continue;

         ConcaveHullVariables subHullVariables = new ConcaveHullVariables();
         List<QuadEdge> subOrderedBorderEdges = subHullVariables.orderedBorderEdges;
         Set<QuadEdge> subBorderEdges = subHullVariables.borderEdges;
         Set<QuadEdgeTriangle> subBorderTriangles = subHullVariables.borderTriangles;
         Set<Vertex> subBorderVertices = subHullVariables.borderVertices;
         PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> subSortedByLengthQueue = subHullVariables.sortedByLengthQueue;
         Set<QuadEdge> subConstraintEdges = subHullVariables.constraintEdges;

         subOrderedBorderEdges.addAll(ListWrappingIndexTools.subListInclusive(subHullStartIndex, subHullEndIndex, orderedBorderEdges));
         checkOrderedBorderEdgeListValid(subOrderedBorderEdges);
         subBorderEdges.addAll(subOrderedBorderEdges);
         subBorderEdges.forEach(borderEdge -> subBorderVertices.add(borderEdge.orig()));
         sortedByLengthQueue.stream()
                            .filter(pair -> isBorderEdge(pair.getLeft(), subBorderEdges))
                            .forEach(subSortedByLengthQueue::add);
         subSortedByLengthQueue.forEach(pair -> subBorderTriangles.add(pair.getRight()));
         subConstraintEdges.addAll(variables.constraintEdges);

         dividedHullVariables.add(subHullVariables);

      } while (ListWrappingIndexTools.next(previousEndIndex, orderedBorderEdges) != startIndex);

      return dividedHullVariables;
   }

   private static void replaceOneEdgeWithTwoInOrderedList(List<QuadEdge> orderedBorderEdges, QuadEdge edgeToReplace, QuadEdge newEdge1, QuadEdge newEdge2)
   {
      int indexOfEdgeToReplace = orderedBorderEdges.indexOf(edgeToReplace);
      if (indexOfEdgeToReplace == -1)
      {
         edgeToReplace = edgeToReplace.sym();
         indexOfEdgeToReplace = orderedBorderEdges.indexOf(edgeToReplace);
      }
      if (indexOfEdgeToReplace == -1)
         throw new RuntimeException("Did not find edge to remove in the ordered border edge list.");

      Vertex previousVertex = edgeToReplace.orig();
      Vertex nextVertex = edgeToReplace.dest();

      QuadEdge firstEdge;
      QuadEdge secondEdge;
      
      if (doesVertexBelongToQuadEdge(previousVertex, newEdge1))
      {
         firstEdge = newEdge1.orig() == previousVertex ? newEdge1 : newEdge1.sym();
         secondEdge = newEdge2.dest() == nextVertex ? newEdge2 : newEdge2.sym();
         if (!doesVertexBelongToQuadEdge(nextVertex, newEdge2))
            throw new RuntimeException("newEdge2 is not connected to nextVertex.");
      }
      else if (doesVertexBelongToQuadEdge(previousVertex, newEdge2))
      {
         firstEdge = newEdge2.orig() == previousVertex ? newEdge2 : newEdge2.sym();
         secondEdge = newEdge1.dest() == nextVertex ? newEdge1 : newEdge1.sym();
         if (!doesVertexBelongToQuadEdge(nextVertex, newEdge1))
            throw new RuntimeException("newEdge1 is not connected to nextVertex.");
      }
      else
      {
         throw new RuntimeException("newEdge1 is not connected to either previousVertex or nextVertex.");
      }

      orderedBorderEdges.set(indexOfEdgeToReplace, firstEdge);
      orderedBorderEdges.add(indexOfEdgeToReplace + 1, secondEdge);
      checkOrderedBorderEdgeListValid(orderedBorderEdges);
   }

   private static void replaceTwoEdgesWithOneInOrderedList(List<QuadEdge> orderedBorderEdges, QuadEdge edgeToReplace1, QuadEdge edgeToReplace2, QuadEdge newEdge)
   {
      int firstEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace1);
      if (firstEdgeIndex == -1)
      {
         edgeToReplace1 = edgeToReplace1.sym();
         firstEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace1);
      }
      if (firstEdgeIndex == -1)
         throw new RuntimeException("Did not find the first edge to remove in the ordered border edge list.");
      int secondEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace2);
      if (secondEdgeIndex == -1)
      {
         edgeToReplace2 = edgeToReplace2.sym();
         secondEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace2);
      }
      if (secondEdgeIndex == -1)
         throw new RuntimeException("Did not find the second edge to remove in the ordered border edge list.");

      if (ListWrappingIndexTools.minDistanceExclusive(firstEdgeIndex, secondEdgeIndex, orderedBorderEdges) != 0)
         throw new RuntimeException("The two edges are not connected");

      Vertex previousVertex;
      Vertex nextVertex;

      if (ListWrappingIndexTools.previous(secondEdgeIndex, orderedBorderEdges) == firstEdgeIndex)
      {
         previousVertex = edgeToReplace1.orig();
         nextVertex = edgeToReplace2.dest();
      }
      else
      {
         previousVertex = edgeToReplace2.orig();
         nextVertex = edgeToReplace1.dest();
      }

      QuadEdge edgeToInsert = newEdge.orig() == previousVertex ? newEdge : newEdge.sym();
      if (edgeToInsert.orig() != previousVertex || edgeToInsert.dest() != nextVertex)
         throw new RuntimeException("The new edge to insert is not properly connected to the list.");

      orderedBorderEdges.set(firstEdgeIndex, edgeToInsert);
      orderedBorderEdges.remove(secondEdgeIndex);
      checkOrderedBorderEdgeListValid(orderedBorderEdges);
   }

   private static boolean doesVertexBelongToQuadEdge(Vertex vertex, QuadEdge edge)
   {
      return edge.orig() == vertex || edge.dest() == vertex;
   }

   private static void checkOrderedBorderEdgeListValid(List<QuadEdge> orderedBorderEdges)
   {
      for (int edgeIndex = 0; edgeIndex < orderedBorderEdges.size(); edgeIndex++)
      {
         Vertex currentDest = orderedBorderEdges.get(edgeIndex).dest();
         Vertex nextOrig = ListWrappingIndexTools.getNext(edgeIndex, orderedBorderEdges).orig();

         if (currentDest != nextOrig)
            throw new RuntimeException("Ordered border edge list is corrupted.");
      }
   }

   private static int numberOfBorderEdges(QuadEdgeTriangle triangle, Set<QuadEdge> borderEdges)
   {
      int numberOfBorderEdges = 0;
      for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
      {
         QuadEdge edge = triangle.getEdge(edgeIndex);
         // Need to check the opposite of the edge too (edge != edge.sym())
         if (isBorderEdge(edge, borderEdges))
            numberOfBorderEdges++;
      }
      return numberOfBorderEdges;
   }

   private static int numberOfBorderVertices(QuadEdgeTriangle triangle, Set<Vertex> borderVertices)
   {
      int numberOfBorderVertices = 0;
      for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
      {
         Vertex vertex = triangle.getVertex(vertexIndex);
         if (borderVertices.contains(vertex))
            numberOfBorderVertices++;
      }
      return numberOfBorderVertices;
   }

   private static boolean isBorderEdge(QuadEdge edge, Set<QuadEdge> borderEdges)
   {
      return borderEdges.contains(edge) || borderEdges.contains(edge.sym());
   }

   private static boolean isConstraintEdge(QuadEdge edge, Set<QuadEdge> constraintEdges)
   {
      return constraintEdges.contains(edge) || constraintEdges.contains(edge.sym());
   }

   private static int indexOfVertexOppositeToEdge(int edgeIndex)
   {
      if (edgeIndex < 0 || edgeIndex > 2)
         throw new RuntimeException("Bad edge index: " + edgeIndex);
      return QuadEdgeTriangle.nextIndex(QuadEdgeTriangle.nextIndex(edgeIndex));
   }

   private static class QuadEdgeComparator implements Comparator<Pair<QuadEdge, QuadEdgeTriangle>>
   {
      private final Map<QuadEdge, Double> map = new HashMap<>();

      @Override
      public int compare(Pair<QuadEdge, QuadEdgeTriangle> pair1, Pair<QuadEdge, QuadEdgeTriangle> pair2)
      {
         double length1 = getEdgeLength(pair1.getKey());
         double length2 = getEdgeLength(pair2.getKey());
         if (length1 < length2)
            return 1;
         else if (length1 == length2)
            return 0;
         else
            return -1;
      }

      private double getEdgeLength(QuadEdge edge)
      {
         Double length = map.get(edge);
         if (length == null)
         {
            length = edge.getLength();
            map.put(edge, length);
            map.put(edge.sym(), length);
         }
         return length;
      }
   }

   public static class ConcaveHullFactoryResult
   {
      private final ConcaveHullCollection concaveHullCollection = new ConcaveHullCollection();
      private final List<QuadEdgeTriangle> allTriangles = new ArrayList<>();
      private final List<ConcaveHullVariables> intermediateVariables = new ArrayList<>();

      public ConcaveHullFactoryResult()
      {
      }

      /** @return the set of edges defining the resulting concave hull. */
      public ConcaveHullCollection getConcaveHullCollection()
      {
         return concaveHullCollection;
      }

      /** @return all the triangles resulting from the Delaunay triangulation. */
      public List<QuadEdgeTriangle> getAllTriangles()
      {
         return allTriangles;
      }

      /** @return the intermediate variables used internally by the factory to compute a concave hull. */
      public List<ConcaveHullVariables> getIntermediateVariables()
      {
         return intermediateVariables;
      }
   }

   public static class ConcaveHullVariables
   {
      private final Set<Vertex> borderVertices = new HashSet<>();
      private final Set<QuadEdge> borderEdges = new HashSet<>();
      private final List<QuadEdge> orderedBorderEdges = new ArrayList<>();
      private final Set<QuadEdgeTriangle> borderTriangles = new HashSet<>();
      private final QuadEdgeComparator quadEdgeComparator = new QuadEdgeComparator();
      private final PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = new PriorityQueue<>(quadEdgeComparator);
      private final Set<QuadEdge> constraintEdges = new HashSet<>();

      public ConcaveHullVariables()
      {
      }

      /** @return vertices of the concave hull. */
      public Set<Vertex> getBorderVertices()
      {
         return borderVertices;
      }

      public Set<QuadEdge> getBorderEdges()
      {
         return borderEdges;
      }

      public List<QuadEdge> getOrderedBorderEdges()
      {
         return orderedBorderEdges;
      }

      /** @return triangles with at least one edge that belongs to the concave hull. */
      public Set<QuadEdgeTriangle> getBorderTriangles()
      {
         return borderTriangles;
      }

      /** @return sorted queue from longest to shortest edges with their triangle. */
      public PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> getSortedByLengthQueue()
      {
         return sortedByLengthQueue;
      }

      /** @return the edges resulting from the constraint segments of the triangulation. */
      public Set<QuadEdge> getConstraintEdges()
      {
         return constraintEdges;
      }
   }
}
