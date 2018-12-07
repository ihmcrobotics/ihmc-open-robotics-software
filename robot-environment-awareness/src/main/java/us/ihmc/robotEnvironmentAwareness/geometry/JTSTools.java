package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullVariables;

/**
 * This class provides tools to convert between the JTS and Euclid geometry types.
 * <p>
 * It assumes that the JTS geometries are in 2D.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class JTSTools
{
   public static MultiPoint point2DsToMultiPoint(Collection<? extends Point2DReadOnly> points)
   {
      return new GeometryFactory().createMultiPoint(point2DsToCoordinates(points));
   }

   public static MultiLineString createMultiLineString(List<? extends LineSegment2DReadOnly> lineSegments)
   {
      return createMultiLineString(lineSegments, 1.0e-5);
   }

   public static MultiLineString createMultiLineString(List<? extends LineSegment2DReadOnly> lineSegments, double epsilon)
   {
      if (lineSegments == null)
         return null;

      List<LineString> lineStrings = lineSegments.stream().map(JTSTools::lineSegment2DToLineString).collect(Collectors.toList());

      GeometryFactory geometryFactory = new GeometryFactory();

      List<LineString> processedLineStrings = new ArrayList<>();

      while (!lineStrings.isEmpty())
      {
         LineString lineStringToProcess = lineStrings.remove(0);
         List<Coordinate> coordinatesToProcess = new ArrayList<>(Arrays.asList(lineStringToProcess.getCoordinates()));

         boolean hasBeenModified = true;

         while (hasBeenModified && !lineStrings.isEmpty())
         {
            for (int i = 0; i < lineStrings.size(); i++)
            {
               LineString candidateLineString = lineStrings.get(i);
               List<Coordinate> candidateCoordinates = new ArrayList<>(Arrays.asList(candidateLineString.getCoordinates()));

               boolean concatenate = false;

               if (lineStringToProcess.getEndPoint().equalsExact(candidateLineString.getStartPoint(), epsilon))
               {
                  concatenate = true;
               }
               else if (lineStringToProcess.getEndPoint().equalsExact(candidateLineString.getEndPoint(), epsilon))
               {
                  concatenate = true;
                  Collections.reverse(candidateCoordinates);
               }
               else if (lineStringToProcess.getStartPoint().equalsExact(candidateLineString.getEndPoint(), epsilon))
               {
                  concatenate = true;
                  Collections.reverse(coordinatesToProcess);
                  Collections.reverse(candidateCoordinates);
               }
               else if (lineStringToProcess.getStartPoint().equalsExact(candidateLineString.getStartPoint(), epsilon))
               {
                  concatenate = true;
                  Collections.reverse(coordinatesToProcess);
               }

               if (concatenate)
               {
                  lineStrings.remove(i);
                  candidateCoordinates.remove(0);
                  coordinatesToProcess.addAll(candidateCoordinates);
                  lineStringToProcess = geometryFactory.createLineString(coordinatesToProcess.toArray(new Coordinate[coordinatesToProcess.size()]));
                  hasBeenModified = true;
                  break;
               }
               else
               {
                  hasBeenModified = false;
               }
            }
         }

         processedLineStrings.add(lineStringToProcess);
      }

      return geometryFactory.createMultiLineString(processedLineStrings.toArray(new LineString[processedLineStrings.size()]));
   }

   public static LineString lineSegment2DToLineString(LineSegment2DReadOnly lineSegment)
   {
      return new GeometryFactory().createLineString(point2DsToCoordinates(lineSegment.getFirstEndpoint(), lineSegment.getSecondEndpoint()));
   }

   public static Coordinate[] point2DsToCoordinates(Point2DReadOnly... points)
   {
      return Stream.of(points).map(JTSTools::point2DToCoordinate).toArray(Coordinate[]::new);
   }

   public static Coordinate[] point2DsToCoordinates(Collection<? extends Point2DReadOnly> points)
   {
      return points.stream().map(JTSTools::point2DToCoordinate).toArray(Coordinate[]::new);
   }

   public static Coordinate point2DToCoordinate(Point2DReadOnly point)
   {
      return new Coordinate(point.getX(), point.getY());
   }

   public static LineSegment2D quadEdgeToLineSegment2D(QuadEdge quadEdge)
   {
      return new LineSegment2D(vertexToPoint2D(quadEdge.orig()), vertexToPoint2D(quadEdge.dest()));
   }

   public static List<LineSegment2D> quadEdgesToLineSegment2Ds(Collection<QuadEdge> quadEdges)
   {
      return quadEdges.stream().map(JTSTools::quadEdgeToLineSegment2D).collect(Collectors.toList());
   }

   public static LineSegment3D quadEdgeToLineSegment3D(QuadEdge quadEdge)
   {
      return new LineSegment3D(vertexToPoint3D(quadEdge.orig()), vertexToPoint3D(quadEdge.dest()));
   }

   public static List<LineSegment3D> quadEdgesToLineSegment3Ds(Collection<QuadEdge> quadEdges)
   {
      return quadEdges.stream().map(JTSTools::quadEdgeToLineSegment3D).collect(Collectors.toList());
   }

   public static Triangle3D quadEdgeTriangleToTriangle(QuadEdgeTriangle quadEdgeTriangle)
   {
      return new Triangle3D(vertexToPoint3D(quadEdgeTriangle.getVertex(0)), vertexToPoint3D(quadEdgeTriangle.getVertex(1)),
                            vertexToPoint3D(quadEdgeTriangle.getVertex(2)));
   }

   public static List<Triangle3D> quadEdgeTrianglesToTriangles(Collection<QuadEdgeTriangle> quadEdgeTriangles)
   {
      return quadEdgeTriangles.stream().map(JTSTools::quadEdgeTriangleToTriangle).collect(Collectors.toList());
   }

   public static Point2D vertexToPoint2D(Vertex vertex)
   {
      return coordinateToPoint2D(vertex.getCoordinate());
   }

   public static Point3D vertexToPoint3D(Vertex vertex)
   {
      return coordinateToPoint3D(vertex.getCoordinate());
   }

   public static Point2D pointToPoint2D(Point point)
   {
      return coordinateToPoint2D(point.getCoordinate());
   }

   public static Point3D pointToPoint3D(Point point)
   {
      return coordinateToPoint3D(point.getCoordinate());
   }

   public static Point2D coordinateToPoint2D(Coordinate coordinate)
   {
      return new Point2D(coordinate.x, coordinate.y);
   }

   public static Point3D coordinateToPoint3D(Coordinate coordinate)
   {
      return new Point3D(coordinate.x, coordinate.y, 0.0);
   }

   public static List<Point3D> extractBorderVerticesInWorld(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {
      return concaveHullFactoryResult.getIntermediateVariables().stream().flatMap(vars -> vars.getBorderVertices().stream()).map(JTSTools::vertexToPoint3D)
                                     .peek(transformToWorld::transform).collect(Collectors.toList());
   }

   public static List<LineSegment3D> extractBorderEdgesInWorld(ConcaveHullVariables concaveHullVariables, RigidBodyTransform transformToWorld)
   {
      return concaveHullVariables.getBorderEdges().stream().map(JTSTools::quadEdgeToLineSegment3D).peek(segment -> segment.applyTransform(transformToWorld))
                                 .collect(Collectors.toList());
   }

   public static List<LineSegment3D> extractBorderEdgesInWorld(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {

      return concaveHullFactoryResult.getIntermediateVariables().stream().flatMap(vars -> vars.getBorderEdges().stream()).map(JTSTools::quadEdgeToLineSegment3D)
                                     .peek(segment -> segment.applyTransform(transformToWorld)).collect(Collectors.toList());
   }

   public static List<LineSegment3D> extractOrderedBorderEdgesInWorld(ConcaveHullVariables concaveHullVariables, RigidBodyTransform transformToWorld)
   {
      return concaveHullVariables.getOrderedBorderEdges().stream().map(JTSTools::quadEdgeToLineSegment3D)
                                 .peek(segment -> segment.applyTransform(transformToWorld)).collect(Collectors.toList());
   }

   public static List<LineSegment3D> extractOrderedBorderEdgesInWorld(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {
      return concaveHullFactoryResult.getIntermediateVariables().stream().flatMap(vars -> vars.getOrderedBorderEdges().stream())
                                     .map(JTSTools::quadEdgeToLineSegment3D).peek(segment -> segment.applyTransform(transformToWorld))
                                     .collect(Collectors.toList());
   }

   public static List<Triangle3D> extractBorderTrianglesInWorld(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {
      return concaveHullFactoryResult.getIntermediateVariables().stream().flatMap(vars -> vars.getBorderTriangles().stream())
                                     .map(JTSTools::quadEdgeTriangleToTriangle).peek(triangle -> triangle.applyTransform(transformToWorld))
                                     .collect(Collectors.toList());
   }

   public static List<Triangle3D> extractAllTrianglesInWorld(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {
      return concaveHullFactoryResult.getAllTriangles().stream().map(JTSTools::quadEdgeTriangleToTriangle)
                                     .peek(triangle -> triangle.applyTransform(transformToWorld)).collect(Collectors.toList());
   }

   public static List<LineSegment3D> extractConstraintEdges(ConcaveHullFactoryResult concaveHullFactoryResult, RigidBodyTransform transformToWorld)
   {
      return concaveHullFactoryResult.getIntermediateVariables().stream().flatMap(vars -> vars.getConstraintEdges().stream())
                                     .map(JTSTools::quadEdgeToLineSegment3D).peek(segment -> segment.applyTransform(transformToWorld))
                                     .collect(Collectors.toList());
   }
}
