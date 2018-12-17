package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

   public class VisibilityGraphEdge implements EpsilonComparable<VisibilityGraphEdge>
   {
      private VisibilityGraphNode sourceNode;
      private VisibilityGraphNode targetNode;

      public VisibilityGraphEdge(VisibilityGraphNode source, VisibilityGraphNode target)
      {
         this.sourceNode = source;
         this.targetNode = target;
      }
      
      public VisibilityGraphNode getSourceNode()
      {
         return sourceNode;
      }
      
      public VisibilityGraphNode getTargetNode()
      {
         return targetNode;
      }

      public ConnectionPoint3D getSourcePoint()
      {
         return sourceNode.getPointInWorld();
      }

      public ConnectionPoint3D getTargetPoint()
      {
         return targetNode.getPointInWorld();
      }

      public Point2D getSourcePoint2D()
      {
         return new Point2D(sourceNode.getPointInWorld());
      }

      public Point2D getTargetPoint2D()
      {
         return new Point2D(targetNode.getPointInWorld());
      }

      public double distanceSquared(Point3DReadOnly query)
      {
         return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query, sourceNode.getPointInWorld(), targetNode.getPointInWorld());
      }

      public double percentageAlongConnection(Point3DReadOnly query)
      {
         return EuclidGeometryTools.percentageAlongLineSegment3D(query, sourceNode.getPointInWorld(), targetNode.getPointInWorld());
      }

      public ConnectionPoint3D orthogonalProjection(Point3DReadOnly pointToProject, int regionId)
      {
         Point3D projection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, sourceNode.getPointInWorld(), targetNode.getPointInWorld());
         return new ConnectionPoint3D(projection, regionId);
      }

      public ConnectionPoint3D getPointGivenPercentage(double percentage, int regionId)
      {
         Point3D result = new Point3D();
         result.interpolate(sourceNode.getPointInWorld(), targetNode.getPointInWorld(), percentage);
         return new ConnectionPoint3D(result, regionId);
      }

      public void flip()
      {
         VisibilityGraphNode temp = sourceNode;
         sourceNode = targetNode;
         targetNode = temp;
      }

      public VisibilityGraphNode getOppositePoint(VisibilityGraphNode point)
      {
         if (point.equals(sourceNode))
            return targetNode;
         else if (point.equals(targetNode))
            return sourceNode;
         return null;
      }

      public double length()
      {
         return sourceNode.distance(targetNode);
      }

      public double lengthSquared()
      {
         return sourceNode.distanceSquared(targetNode);
      }

      @Override
      public boolean epsilonEquals(VisibilityGraphEdge other, double epsilon)
      {
         return sourceNode.epsilonEquals(other.sourceNode, epsilon) && targetNode.epsilonEquals(other.targetNode, epsilon)
               || sourceNode.epsilonEquals(other.targetNode, epsilon) && targetNode.epsilonEquals(other.sourceNode, epsilon);
      }

      @Override
      public int hashCode()
      {
         return sourceNode.hashCode() + targetNode.hashCode();
      }

      @Override
      public boolean equals(Object obj)
      {
         try
         {
            return equals((Connection) obj);
         }
         catch (ClassCastException e)
         {
            return false;
         }
      }

      public boolean equals(VisibilityGraphEdge other)
      {
         if (other == null)
            return false;
         else
            return (sourceNode.equals(other.sourceNode) && targetNode.equals(other.targetNode)) || (sourceNode.equals(other.targetNode) && targetNode.equals(other.sourceNode));
      }

      @Override
      public String toString()
      {
         return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(sourceNode.getPointInWorld()) + ", target = " + EuclidCoreIOTools.getTuple3DString(targetNode.getPointInWorld());
      }
   } 
   
