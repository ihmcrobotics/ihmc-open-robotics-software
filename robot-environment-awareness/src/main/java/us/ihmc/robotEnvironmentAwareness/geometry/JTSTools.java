package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullVariables;

public class JTSTools
{
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
      return new Point2D(vertex.getX(), vertex.getY());
   }

   public static Point3D vertexToPoint3D(Vertex vertex)
   {
      return new Point3D(vertex.getX(), vertex.getY(), 0.0);
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
