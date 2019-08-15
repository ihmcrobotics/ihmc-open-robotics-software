package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class VisibilityGraphEdge implements EpsilonComparable<VisibilityGraphEdge>, LineSegment3DReadOnly
{
   private final VisibilityGraphNode sourceNode;
   private final VisibilityGraphNode targetNode;

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

   public ConnectionPoint3D getSourcePointInWorld()
   {
      return sourceNode.getPointInWorld();
   }

   public ConnectionPoint3D getTargetPointInWorld()
   {
      return targetNode.getPointInWorld();
   }

   public void registerEdgeWithNodes()
   {
      sourceNode.addEdge(this);
      targetNode.addEdge(this);
   }

   public double percentageAlongConnection(Point3DReadOnly query)
   {
      return EuclidGeometryTools.percentageAlongLineSegment3D(query, sourceNode.getPointInWorld(), targetNode.getPointInWorld());
   }

   @Override
   public boolean epsilonEquals(VisibilityGraphEdge other, double epsilon)
   {
      return sourceNode.epsilonEquals(other.sourceNode, epsilon) && targetNode.epsilonEquals(other.targetNode, epsilon)
            || sourceNode.epsilonEquals(other.targetNode, epsilon) && targetNode.epsilonEquals(other.sourceNode, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == null)
         return false;

      if (object instanceof VisibilityGraphEdge)
      {
         return equals((VisibilityGraphEdge) object);
      }
      return false;
   }

   public boolean equals(VisibilityGraphEdge other)
   {
      if (other == null)
         return false;
      else
         return (sourceNode.equals(other.sourceNode) && targetNode.equals(other.targetNode))
               || (sourceNode.equals(other.targetNode) && targetNode.equals(other.sourceNode));
   }

   @Override
   public String toString()
   {
      return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(sourceNode.getPointInWorld()) + ", target = "
            + EuclidCoreIOTools.getTuple3DString(targetNode.getPointInWorld());
   }

   @Override
   public Point3DReadOnly getFirstEndpoint()
   {
      return sourceNode.getPointInWorld();
   }

   @Override
   public Point3DReadOnly getSecondEndpoint()
   {
      return targetNode.getPointInWorld();
   }
}
